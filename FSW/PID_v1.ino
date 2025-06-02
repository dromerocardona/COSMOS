#include <Arduino_LSM6DS3.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <ScioSense_ENS220.h>
#include <ens220.h>
#include <utils.h>
#include <SPI.h>
#include <SD.h>
#include <Arduino.h>
#include "i2c_interface.h"
#include <Servo.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include <cmath>

// Pin and constant definitions
#define SD_CS_PIN 6
#define SERVO_PIN A3
#define I2C_ADDRESS 0x20
#define MAG1_I2C_ADDRESS 0x1C
#define MAG2_I2C_ADDRESS 0x1E  // Assuming second magnetometer address
#define TEAM_ID "3195"
#define PULSECOMS A4
#define BUFF_SIZE 20

const int pin = A4;
const bool DEBUG = false;

// Global variables
uint8_t txBuf[BUFF_SIZE], crc;
int recState = 0;  // 0 = not recording, 1 = recording
unsigned long highStartTime = 0;
bool wasHigh = false;

// Sensor objects
LIS3MDL mag;
LSM6 imu;
ScioSense::ENS220 ens220;
I2cInterface i2c_1;
Adafruit_LIS3MDL lis3mdl;
Adafruit_LIS3MDL lis3mdl_FC;
Adafruit_LIS3MDL lis3mdl_CAM;
Servo camServo;
File dataFile;
File backupFile;

// Calibration variables
float mx_min = 1e6, my_min = 1e6, mz_min = 1e6;
float mx_max = -1e6, my_max = -1e6, mz_max = -1e6;
float offset_x = 0, offset_y = 0, offset_z = 0;
float scale_x = 1, scale_y = 1, scale_z = 1;
bool calibrated = false;

// Running average variables
float avg_cos = 0;
float avg_sin = 0;
float avgLen = 2;
bool first_update = true;

// PulseComs variables
const int FC_COMS = PULSECOMS;
volatile int pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
const unsigned long pulseTimeout = 500;

// PID variables
float setpoint = 0.0;  // Desired heading (north)
float lastError = 0.0;
float integral = 0.0;
float lastVelError = 0.0;
float velIntegral = 0.0;

// Other variables
unsigned long landedTime = 0;
unsigned long lastOrientationTime = 0;
uint8_t satellites = 0;
float voltageDividerFactor = 0.012089;
float lastTransmissionTime = 0;
char currentTime[9] = "00:00:00";
char gpsTime[9] = "00:00:00";
float mag2X, mag2Y, mag2Z;
float accel2X, accel2Y, accel2Z, gyro2X, gyro2Y, gyro2Z;
char lastCommand[32];
unsigned int packetCount = 0;
bool telemetryEnabled = true;
float timeDifference = 0;

// Altitude calculation variables
float apogeeAltitude = 0.0;
float maxAltitude = 0;
bool releaseActivated = false;
bool simulationMode = false;
float simulatedPressure = 0.0;
float receivedPressure = 0.0;
float referencePressure = 1013.25;
const int historySize = 10;
float pressure;
float temperature;
float altitudeHistory[historySize];
float velocityHistory[historySize];
unsigned long timestampHistory[historySize];
float latestVelocity;

// Function Definitions

/** Maps a float value from one range to another */
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/** Implements cascaded PID control for position and velocity */
float cascadePidControl(float currentPosition, float positionSetpoint, float currentVelocity, 
                        float &lastPosError, float &posIntegral,
                        float &lastVelError, float &velIntegral,
                        bool invert) {
    // Outer Loop: Position Controller  this will tune the position. this should be tunned with a predictable response on the secondary loop 
    const float Kp_pos = 1.0f;//tune first  (the cumulative size of p and d will directly determine both the response and instability of the system (same for the inner loop))
    const float Ki_pos = 0.0f;
    const float Kd_pos = 1.5f;//second
    const float rng = 360.0f;

    float posError = positionSetpoint - currentPosition;
    if (posError > 180.0f) posError -= 360.0f;
    else if (posError < -180.0f) posError += 360.0f;

    posIntegral += posError;
    float posDerivative = posError - lastPosError;
    lastPosError = posError;

    float desiredVelocity = Kp_pos * posError + Ki_pos * posIntegral + Kd_pos * posDerivative;
    if (desiredVelocity > rng) desiredVelocity = rng;
    else if (desiredVelocity < -rng) desiredVelocity = -rng;

    // Inner Loop: Velocity Controller  this should be tunned second to stabilize occilation on the position loop by dampening velocity values
    const float Kp_vel = 1.0f;//fune first (tuning above 2 may have a bennificial response effect)
    const float Ki_vel = 0.0f;//will potentialy have a beneficial effect.  values above 0.1 may cause uncontrolled windup
    const float Kd_vel = 0.2f;//tune second  (this could stabilize a large p value)

    float velError = desiredVelocity - currentVelocity;
    velIntegral += velError;
    float velDerivative = velError - lastVelError;
    lastVelError = velError;

    float outputVelocity = Kp_vel * velError + Ki_vel * velIntegral + Kd_vel * velDerivative;

    // Mapping to Servo PWM
    int MMoffset = 10;
    int Maxms = 1595 + MMoffset;
    int Minms = 1355 - MMoffset;
    float servoMicroseconds;
    int Deadzonecenter = 1475;
    int Dzoffset = -10;

    if (!invert) {
        if (outputVelocity > 0) {
            servoMicroseconds = mapFloat(outputVelocity, 0, rng, Deadzonecenter + Dzoffset, Maxms);
        } else if (outputVelocity < 0) {
            servoMicroseconds = mapFloat(outputVelocity, -rng, 0, Minms, Deadzonecenter - Dzoffset);
        } else {
            servoMicroseconds = Deadzonecenter;
        }
    } else {
        if (outputVelocity > 0) {
            servoMicroseconds = mapFloat(outputVelocity, 0, rng, Minms, Deadzonecenter - Dzoffset);
        } else if (outputVelocity < 0) {
            servoMicroseconds = mapFloat(outputVelocity, -rng, 0, Deadzonecenter + Dzoffset, Maxms);
        } else {
            servoMicroseconds = Deadzonecenter;
        }
    }
    return servoMicroseconds;
}

/** Debug checkpoint logging */
void debugCheckpoint(const char *message) {
    static int checkpoint = 0;
    if (DEBUG) {
        checkpoint++;
        Serial.print("Checkpoint ");
        Serial.print(checkpoint);
        Serial.print(": ");
        Serial.println(message);
    }
}

/** Calculates altitude from pressure */
float calculateAltitude(float pressure) {
    const float temperatureLapseRate = 0.0065;
    const float seaLevelTemperature = 288.15;
    const float gasConstant = 8.3144598;
    const float molarMass = 0.0289644;
    const float gravity = 9.80665;
    float altitude = (seaLevelTemperature / temperatureLapseRate) * 
                     (1 - pow((pressure / referencePressure), (gasConstant * temperatureLapseRate) / (gravity * molarMass)));
    return altitude;
}

/** Updates altitude history array */
void updateAltitudeHistory(float altitudeHistory[], unsigned long timestampHistory[], float newAltitude, int size) {
    for (int i = size - 1; i > 0; i--) {
        altitudeHistory[i] = altitudeHistory[i - 1];
        timestampHistory[i] = timestampHistory[i - 1];
    }
    altitudeHistory[0] = newAltitude;
    timestampHistory[0] = millis();
}

/** Updates velocity history based on altitude changes */
void updateVelocityHistory(float altitudeHistory[], float velocityHistory[], unsigned long timestampHistory[], int size) {
    for (int i = size - 1; i > 0; i--) {
        velocityHistory[i] = velocityHistory[i - 1];
    }
    if (timestampHistory[1] != 0) {
        float deltaAltitude = altitudeHistory[0] - altitudeHistory[1];
        float deltaTime = (timestampHistory[0] - timestampHistory[1]) / 1000.0;
        velocityHistory[0] = deltaAltitude / deltaTime;
    } else {
        velocityHistory[0] = 0;
    }
}

/** Calculates average of an array */
float avg(float arr[], int size) {
    float sum = 0;
    for (int i = 0; i < size; i++) sum += arr[i];
    return sum / size;
}

/** Setup for ENS220 pressure sensor single-shot measurement */
void SingleShotMeasure_setup() {
  // Start the communication, confirm the device PART_ID, and read the device UID
  i2c_1.begin(Wire, I2C_ADDRESS);

  while (ens220.begin(&i2c_1) != true) {
    Serial.println("Waiting for I2C to start");
    delay(1000);
  }

  Serial.print("Device UID: ");
  Serial.println(ens220.getUID(), HEX);

  // Choose the desired configuration of the sensor. In this example we will use the Lowest Noise settings from the datasheet
  ens220.setDefaultConfiguration();
  ens220.setPressureConversionTime(ENS220::PressureConversionTime::T_16_4);
  ens220.setOversamplingOfPressure(ENS220::Oversampling::N_8);
  ens220.setOversamplingOfTemperature(ENS220::Oversampling::N_8);
  ens220.setPressureTemperatureRatio(ENS220::PressureTemperatureRatio::PT_1);
  ens220.setStandbyTime(ENS220::StandbyTime::OneShotOperation);
  ens220.setPressureDataPath(ENS220::PressureDataPath::Direct);

  // Write the desired configuration into the sensor
  ens220.writeConfiguration();
}

/** Loop for ENS220 pressure sensor measurement */
void SingleShotMeasure_loop() {
  // Start single shot measurement
  ens220.singleShotMeasure(ENS220::Sensor::TemperatureAndPressure);

  // Wait until the measurement is ready
  ens220.waitSingleShot();

  // Check the DATA_STAT from the sensor. If data is available, it reads it
  auto result = ens220.update();

  if (result == ENS220::Result::Ok) {
    if (hasFlag(ens220.getDataStatus(), ENS220::DataStatus::PressureReady) && hasFlag(ens220.getDataStatus(), ENS220::DataStatus::TemperatureReady)) {
      // Send the values that were collected during the ens220.update()
      //Serial.print("P[hPa]:");
      pressure = ens220.getPressureHectoPascal();
      //Serial.print(pressure);
      //Serial.print("\tT[C]:");
      temperature = ens220.getTempCelsius();
      //Serial.println(temperature);
    }
  }
}

/** Collects magnetometer calibration data */
void collectCalibrationData() {
    mag.read();
    mx_min = min(mx_min, mag.m.x);
    mx_max = max(mx_max, mag.m.x);
    my_min = min(my_min, mag.m.y);
    my_max = max(my_max, mag.m.y);
    mz_min = min(mz_min, mag.m.z);
    mz_max = max(mz_max, mag.m.z);
}

/** Computes magnetometer calibration parameters */
void computeCalibration() {
    offset_x = (mx_max + mx_min) / 2;
    offset_y = (my_max + my_min) / 2;
    offset_z = (mz_max + mz_min) / 2;
    scale_x = 2 / (mx_max - mx_min);
    scale_y = 2 / (my_max - my_min);
    scale_z = 2 / (mz_max - mz_min);
    calibrated = true;
}

/** Computes tilt-compensated heading */
float computeTiltCompensatedHeading() {
    mag.read();
    imu.read();
    LIS3MDL::vector<float> m = {
        (mag.m.x - offset_x) * scale_x,
        (mag.m.y - offset_y) * scale_y,
        (mag.m.z - offset_z) * scale_z
    };
    LIS3MDL::vector<int16_t> a = { -imu.a.y, imu.a.x, imu.a.z };
    LIS3MDL::vector<float> E, N;
    LIS3MDL::vector_cross(&m, &a, &E);
    LIS3MDL::vector_normalize(&E);
    LIS3MDL::vector_cross(&a, &E, &N);
    LIS3MDL::vector_normalize(&N);
    LIS3MDL::vector<int> z_axis = { 0, 0, 1 };
    float heading = atan2(LIS3MDL::vector_dot(&E, &z_axis), LIS3MDL::vector_dot(&N, &z_axis)) * 180.0 / M_PI;
    if (heading < 0) heading += 360;
    return heading;
}

/** Updates running average for heading smoothing */
void updateRunningAverage(float heading, float &smoothed_heading) {
    float heading_rad = heading * M_PI / 180.0;
    if (first_update) {
        avg_cos = cos(heading_rad);
        avg_sin = sin(heading_rad);
        smoothed_heading = heading;
        first_update = false;
    } else {
        float alpha = (avgLen - 1.0) / avgLen;
        avg_cos = alpha * avg_cos + (1 - alpha) * cos(heading_rad);
        avg_sin = alpha * avg_sin + (1 - alpha) * sin(heading_rad);
        smoothed_heading = atan2(avg_sin, avg_cos) * 180.0 / M_PI;
        if (smoothed_heading < 0) smoothed_heading += 360;
    }
}

/** Interrupt handler for pulse communication */
void readPulseComs() {
    unsigned long currentTime = millis();
    if (currentTime - lastPulseTime < pulseTimeout) {
        pulseCount++;
    } else {
        pulseCount = 1;
    }
    lastPulseTime = currentTime;
    if (pulseCount >= 3) {
        Serial.println("Three HIGH pulses detected!");
        pulseCount = 0;
    }
}

/** Calculates CRC for camera control buffer */
uint8_t calcCrc(uint8_t *buf, uint8_t numBytes) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < numBytes; i++) {
        crc = crc8_calc(crc, buf[i], 0x07);
    }
    return crc;
}

/** CRC-8 calculation function */
uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly) {
    crc ^= a;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x80) crc = (crc << 1) ^ poly;
        else crc <<= 1;
    }
    return crc;
}

/** Setup function */
void setup() {
    Serial.begin(115200);
    delay(5000);
    Serial.println("hi");

    Wire.begin();
    Wire.setClock(400000);
    Serial1.begin(115200);
    delay(3000);

    // Initialize camera control buffer
    txBuf[0] = 0xCC;
    txBuf[1] = 0x01;
    txBuf[2] = 0x01;
    txBuf[3] = calcCrc(txBuf, 3);

    pinMode(pin, INPUT_PULLUP);

    // Initialize magnetometer
    if (!mag.init()) {
        Serial.println("Failed to detect and initialize LIS3MDL magnetometer!");
    }
    mag.enableDefault();

    // Initialize IMU
    if (!imu.init()) {
        Serial.println("Failed to detect and initialize LSM6 IMU!");
    }
    imu.enableDefault();

    // Attach servo
    camServo.attach(SERVO_PIN);

    // Perform magnetometer calibration
    Serial.println("Please rotate device for 10 seconds to calibrate...");
    unsigned long startTime = millis();
    while (!calibrated) {
        if (millis() - startTime < 10000) {
            collectCalibrationData();
        } else {
            computeCalibration();
        }
        if (millis() - startTime < 5000) {
            camServo.writeMicroseconds(1355);
        } else if (millis() - startTime > 5000) {
            camServo.writeMicroseconds(1595);
        }
        delay(10);
    }

    // Initialize SD card
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD card initialization failed!");
    } else {
        Serial.println("SD card initialized successfully");
    }
    backupFile = SD.open("backup.txt", FILE_WRITE);

    // Initialize first magnetometer
    if (!lis3mdl_FC.begin_I2C(MAG1_I2C_ADDRESS)) {
        Serial.println("Failed to find LIS3MDL #1");
    } else {
        Serial.println("LIS3MDL #1 Found!");
        lis3mdl_FC.setPerformanceMode(LIS3MDL_MEDIUMMODE);
        lis3mdl_FC.setOperationMode(LIS3MDL_CONTINUOUSMODE);
        lis3mdl_FC.setDataRate(LIS3MDL_DATARATE_155_HZ);
        lis3mdl_FC.setRange(LIS3MDL_RANGE_4_GAUSS);
    }

    // Initialize second magnetometer
    if (!lis3mdl_CAM.begin_I2C(MAG2_I2C_ADDRESS)) {
        Serial.println("Failed to find LIS3MDL #2");
    } else {
        Serial.println("LIS3MDL #2 Found!");
        lis3mdl_CAM.setPerformanceMode(LIS3MDL_MEDIUMMODE);
        lis3mdl_CAM.setOperationMode(LIS3MDL_CONTINUOUSMODE);
        lis3mdl_CAM.setDataRate(LIS3MDL_DATARATE_155_HZ);
        lis3mdl_CAM.setRange(LIS3MDL_RANGE_4_GAUSS);
    }

    // Initialize third magnetometer
    if (!lis3mdl.begin_I2C()) {
        Serial.println("Failed to find LIS3MDL chip");
    } else {
        Serial.println("LIS3MDL Found!");
        lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
        lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
        lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
        lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
    }

    // Initialize IMU (Arduino_LSM6DS3)
    if (!IMU.begin()) {
        Serial.println("Error initializing LSM6DS3 #1!");
    } else {
        Serial.print("Accelerometer #1 sample rate = ");
        Serial.print(IMU.accelerationSampleRate());
        Serial.println(" Hz");
        Serial.println("Acceleration in g's");
        Serial.println("X\tY\tZ");
    }

    // Initialize pressure sensor
    SingleShotMeasure_setup();

    // Setup pulse communication interrupt
    pinMode(PULSECOMS, INPUT);
    attachInterrupt(digitalPinToInterrupt(PULSECOMS), readPulseComs, RISING);
}

/** Main loop function */
void loop() {
    // Read sensor data
    mag.read();
    imu.read();

    // Compute heading and smooth it
    float heading = computeTiltCompensatedHeading();
    float smoothed_heading;
    updateRunningAverage(heading, smoothed_heading);

    // Read gyroscope data for velocity
    float gyroX, gyroY, gyroZ;
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyroX, gyroY, gyroZ);
    }

    // Apply cascaded PID control
    float currentVelocity = gyroZ;
    float pwmValue = cascadePidControl(heading, setpoint, currentVelocity, lastError, integral, lastVelError, velIntegral, true);
    camServo.writeMicroseconds((int)pwmValue);

    // Output for monitoring
    Serial.print("360,0,");
    Serial.print(smoothed_heading);
    Serial.print(",");
    Serial.println(heading);

    // Read additional sensor data
    unsigned long missionTime = millis() / 1000;
    sensors_event_t magEvent1;
    lis3mdl.getEvent(&magEvent1);
    sensors_event_t magEvent2;
    lis3mdl_CAM.getEvent(&magEvent2);
    mag2X = magEvent2.magnetic.x;
    mag2Y = magEvent2.magnetic.y;
    mag2Z = magEvent2.magnetic.z;
    float accelX, accelY, accelZ;
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accelX, accelY, accelZ);
    }
    SingleShotMeasure_loop();
    float altitude = calculateAltitude(pressure);

    // Format and log telemetry data
    char telemetry[256];
    snprintf(telemetry, sizeof(telemetry),
             "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,COSMOS",
             altitude, temperature, pressure, 0.0,  // Placeholder for voltage
             gyroX, gyroY, gyroZ, accelX, accelY, accelZ,
             magEvent1.magnetic.x, magEvent1.magnetic.y, magEvent1.magnetic.z);

    dataFile = SD.open("data.txt", FILE_WRITE);
    if (dataFile) {
        dataFile.println(telemetry);
        dataFile.close();
    }
    packetCount++;

    // Handle pulse communications
    int comsState = digitalRead(FC_COMS);
    if (comsState == HIGH) {
        // Add communication handling if needed
    }

    // Update altitude and velocity history
    updateAltitudeHistory(altitudeHistory, timestampHistory, altitude, historySize);
    updateVelocityHistory(altitudeHistory, velocityHistory, timestampHistory, historySize);

    // Camera recording control
    int pinState = digitalRead(pin);
    if (pinState == LOW && recState == 0) {
        Serial1.write(txBuf, 4);
        recState = 1;
        Serial.println("Started Recording");
    } else if (pinState == HIGH && recState == 1) {
        if (!wasHigh) {
            highStartTime = millis();
            wasHigh = true;
        } else if (millis() - highStartTime >= 1000) {
            Serial1.write(txBuf, 4);
            recState = 0;
            Serial.println("Stopped Recording");
            wasHigh = false;
        }
    } else if (pinState == LOW) {
        wasHigh = false;
    }
}
