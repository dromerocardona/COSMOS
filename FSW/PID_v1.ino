// LIBRARY INCLUSIONS
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

//buffer for delaying SD writes to avoid escessive open/close file commands
char buffer[1024];
int bufferIndex = 0;  // Tracks the current position in the buffer

//--Feedback
const int feedbackPin = A5;
const int outputPin = A3;
const int timeout = 100000;  // 0.1 second timeout
float tHigh = 0;
float tLow = 0;
float dutyCycle = 0;
float currentAngle = 0;
//--


const int altitude = 0;
const int gyroX = 0;
const int gyroY = 0;
const int gyroZ = 0;
const int accelX = 0;
const int accelY = 0;
const int accelZ = 0;

//----------------Tilt compensation----------------
LIS3MDL mag;
LSM6 imu;

// Calibration variables (for magnetometer)
float mx_min = 1e6, my_min = 1e6, mz_min = 1e6;
float mx_max = -1e6, my_max = -1e6, mz_max = -1e6;
float offset_x = 0, offset_y = 0, offset_z = 0;
float scale_x = 1, scale_y = 1, scale_z = 1;
bool calibrated = false;

// Running average variables
float avg_cos = 0;         // Average cosine component
float avg_sin = 0;         // Average sine component
float avgLen = 2;          // Smoothing factor (higher = smoother, slower response)
bool first_update = true;  // Flag for initializing the average


void collectCalibrationData() {
  mag.read();
  mx_min = min(mx_min, mag.m.x);
  my_min = min(my_min, mag.m.y);
  mz_min = min(mz_min, mag.m.z);
  mx_max = max(mx_max, mag.m.x);
  my_max = max(my_max, mag.m.y);
  mz_max = max(mz_max, mag.m.z);
}

void computeCalibration() {
  offset_x = (mx_max + mx_min) / 2.0;
  offset_y = (my_max + my_min) / 2.0;
  offset_z = (mz_max + mz_min) / 2.0;

  float rx = (mx_max - mx_min) / 2.0;
  float ry = (my_max - my_min) / 2.0;
  float rz = (mz_max - mz_min) / 2.0;
  float r_avg = (rx + ry + rz) / 3.0;

  scale_x = r_avg / rx;
  scale_y = r_avg / ry;
  scale_z = r_avg / rz;

  calibrated = true;
  Serial.println("Calibration complete.");
}

float computeTiltCompensatedHeading() {
  //mag.read();
  //imu.read();

  // Apply calibration
  LIS3MDL::vector<float> m = {
    (mag.m.x - offset_x) * scale_x,
    (mag.m.y - offset_y) * scale_y,
    (mag.m.z - offset_z) * scale_z
  };
  LIS3MDL::vector<int16_t> a = { -imu.a.y, imu.a.x, imu.a.z };

  // Tilt compensation
  LIS3MDL::vector<float> E, N;
  LIS3MDL::vector_cross(&m, &a, &E);
  LIS3MDL::vector_normalize(&E);
  LIS3MDL::vector_cross(&a, &E, &N);
  LIS3MDL::vector_normalize(&N);

  LIS3MDL::vector<int> z_axis = { 0, 0, 1 };
  float heading = atan2(LIS3MDL::vector_dot(&E, &z_axis),
                        LIS3MDL::vector_dot(&N, &z_axis))
                  * 180.0 / M_PI;
  if (heading < 0) heading += 360;

  return heading;
}

void updateRunningAverage(float heading, float &smoothed_heading) {
  // Convert heading to radians
  float heading_rad = heading * M_PI / 180.0;

  if (first_update) {
    // Initialize with the first heading
    avg_cos = cos(heading_rad);
    avg_sin = sin(heading_rad);
    smoothed_heading = heading;
    first_update = false;
  } else {
    // Update averages using EMA
    float alpha = (avgLen - 1.0) / avgLen;  // Weight for previous average
    avg_cos = alpha * avg_cos + (1 - alpha) * cos(heading_rad);
    avg_sin = alpha * avg_sin + (1 - alpha) * sin(heading_rad);

    // Compute smoothed heading
    smoothed_heading = atan2(avg_sin, avg_cos) * 180.0 / M_PI;
    if (smoothed_heading < 0) smoothed_heading += 360;
  }
}
//-------------------------end tilt comp-----------------------------------

//debugging:
const bool DEBUG = false;

void debugCheckpoint(const char *message) {
  static int checkpoint = 0;  // Static variable retains its value between calls
  if (DEBUG) {
    checkpoint++;  // Increment the checkpoint number
    Serial.print("Checkpoint ");
    Serial.print(checkpoint);  // Print the current checkpoint number
    Serial.print(": ");
    Serial.println(message);  // Print the provided message
  }
}

// PINS AND DEFINITIONS
#define SD_CS_PIN 6   // Chip select pin for SD card
#define SERVO_PIN A3  // Servo pin for GND camera stabilization
//#define FEEDBACK_PIN 9        // Feedback signal pin for servo control
#define I2C_ADDRESS 0x20       // I2C Address for ENS 220
#define MAG1_I2C_ADDRESS 0x1C  // First LIS3MDL address
#define TEAM_ID "3195"
#define PULSECOMS A4
//------------------------------------------run cam---------------
// A Program to Toggle RunCam Split HD recording with Serial UART
#define BUFF_SIZE 20
#define Serial1 Serial1

//const int pin = A4;
uint8_t txBuf[BUFF_SIZE], crc;
int recState = 0;  // 0 = not recording, 1 = recording
unsigned long highStartTime = 0;
bool wasHigh = false;

uint8_t calcCrc(uint8_t *buf, uint8_t numBytes) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < numBytes; i++)
    crc = crc8_calc(crc, buf[i], 0xd5);
  return crc;
}

uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly) {
  crc ^= a;
  for (int ii = 0; ii < 8; ++ii) {
    if (crc & 0x80)
      crc = (crc << 1) ^ poly;
    else
      crc = crc << 1;
  }
  return crc;
}
//------------------------------------------end runcam------------------

ScioSense::ENS220 ens220;
I2cInterface i2c_1;        // Added for ENS220 single-shot mode
Adafruit_LIS3MDL lis3mdl;  // Magnetometer
Servo camServo;
File dataFile;
File backupFile;

// Variables
unsigned long landedTime = 0;
unsigned long lastOrientationTime = 0;
//float lastOrientationX = 0.0, lastOrientationY = 0.0, lastOrientationZ = 0.0;
uint8_t satellites = 0;
float voltageDividerFactor = 0.012089;  // Adjust based on resistor values in voltage divider
float lastTransmissionTime = 0;         // Last time of telemetry transmission
char currentTime[9] = "00:00:00";       // Mission time in "HH:MM:SS"
float magX, magY, magZ;                 // Magnetometer data for second sensor
char lastCommand[32];                   // Last received command
unsigned int packetCount = 0;
bool telemetryEnabled = true;  // Telemetry Control
float timeDifference = 0;      // Time difference between two consecutive interrupts

// Altitude calculation variables
float apogeeAltitude = 0.0;
float maxAltitude = 0;
bool releaseActivated = false;
bool simulationMode = false;
float simulatedPressure = 0.0;
float receivedPressure = 0.0;       // For SIM_ACTIVATE pressure input
float referencePressure = 1013.25;  // Default reference point (sea level)
const int historySize = 10;
float pressure;
float temperature;
float altitudeHistory[historySize];           // Store altitude
float velocityHistory[historySize];           // Store velocity
unsigned long timestampHistory[historySize];  // Store time
float latestVelocity;

// Variables used for PID
float setpoint = 1;     // Desired setpoint
float input = 0.0;      // Current system input
float output = 0.0;     // PID output
float error = 0.0;      // Current error
float lastError = 0.0;  // Previous error
float integral = 0.0;   // tracks cumulative error
float heading;

// PulseComs™
volatile int pulseCount = 0;  // Must be 'volatile' since used in interrupt
volatile unsigned long lastPulseTime = 0;
const unsigned long pulseTimeout = 500;  // Max gap between pulses (adjustable)

///////////////////////// FUNCTIONS /////////////////////////

// PulseComs™
void readPulseComs() {
  unsigned long currentTime = millis();

  if (currentTime - lastPulseTime < pulseTimeout) {
    pulseCount++;  // Count consecutive HIGH pulses
  } else {
    pulseCount = 1;  // Reset count if too much time has passed
  }

  lastPulseTime = currentTime;

  if (pulseCount >= 3) {
    pulseCount = 0;  // Reset after recognition
  }
}

// Function to calculate altitude from pressure
float calculateAltitude(float pressure) {
  // Constants for the barometric formula
  const float temperatureLapseRate = 0.0065;  // Temperature lapse rate in K/m
  const float seaLevelTemperature = 288.15;   // Sea level standard temperature in K
  const float gasConstant = 8.3144598;        // Universal gas constant in J/(mol*K)
  const float molarMass = 0.0289644;          // Molar mass of Earth's air in kg/mol
  const float gravity = 9.80665;              // Acceleration due to gravity in m/s^2

  // Calculate altitude using the barometric formula
  float altitude = (seaLevelTemperature / temperatureLapseRate) * (1 - pow((pressure / referencePressure), (gasConstant * temperatureLapseRate) / (gravity * molarMass)));
  return altitude;
}

// Function to update the altitude history array
void updateAltitudeHistory(float altitudeHistory[], unsigned long timestampHistory[], float newAltitude, int size) {
  // Shift all elements in altitudeHistory and timestampHistory to the right
  for (int i = size - 1; i > 0; i--) {
    altitudeHistory[i] = altitudeHistory[i - 1];
    timestampHistory[i] = timestampHistory[i - 1];
  }
  // Update the first element with the new altitude and current timestamp
  altitudeHistory[0] = newAltitude;
  timestampHistory[0] = millis();
}

// Function to calculate velocity and update the velocity history array
void updateVelocityHistory(float altitudeHistory[], float velocityHistory[], unsigned long timestampHistory[], int size) {
  // Calculate the time difference between the two most recent updates in milliseconds
  unsigned long timeDifferenceMillis = timestampHistory[0] - timestampHistory[1];
  float timeDifferenceSeconds = timeDifferenceMillis / 1000.0;  // Convert to seconds

  latestVelocity = (altitudeHistory[0] - altitudeHistory[1]) / timeDifferenceSeconds;  // Calculate the latest velocity

  // Shift all elements in velocityHistory to the right
  for (int i = size - 1; i > 0; i--) {
    velocityHistory[i] = velocityHistory[i - 1];
  }

  velocityHistory[0] = latestVelocity;  // Update the first element with the latest velocity
}

float avg(float arr[], int size) {
  float sum = 0.0;
  for (int i = 0; i < size; i++) {
    sum += arr[i];
  }
  return sum / size;
}
float lastDerivative = 0.0;  // Previous derivative



float pidControl(float input, float setpoint, float &lastError, float &integral, float &lastDerivative, bool invert, float currentAngle, float expCoefficient = 0.0) {
  // PID tuning parameters
  const float K_proportional = 0.015;
  const float K_integral = 0.000;
  const float K_derivative = 0.6;
  const float K_secondDerivative = 0;
  const float rng = 360.0;

  Serial.println(heading);

  const float maxIntegral = 500;           // Anti-windup clamp
  const float maxDeltaServo = 15;          // Limit how fast servo command changes

  // Angle wrapping
  float adjusted = -currentAngle - (-currentAngle - input);
  if (adjusted > 180.0) adjusted -= 360.0;
  else if (adjusted < -180.0) adjusted += 360.0;

  float error = setpoint - adjusted;
  if (error > 180.0) error -= 360.0;
  else if (error < -180.0) error += 360.0;

  // Integral with anti-windup
  integral += error;
  if (integral > maxIntegral) integral = maxIntegral;
  else if (integral < -maxIntegral) integral = -maxIntegral;

  // Derivative on measurement (input-based to avoid kick)
  float derivative = -(input - lastDerivative);

  // Second derivative term
  float secondDerivative = derivative - (lastError - lastDerivative);

  // Proportional term (optionally exponential)
  float pTerm = (expCoefficient != 0.0)
    ? K_proportional * error * exp(expCoefficient * abs(error))
    : K_proportional * error;

  // Total PID output
  float output = pTerm + K_integral * integral + K_derivative * derivative + K_secondDerivative * secondDerivative;

  // Update previous values
  lastDerivative = input;
  lastError = error;

  // Servo control setup
  float MMoffset = 150;
  float Maxms = 1595 + MMoffset;
  float Minms = 1355 - MMoffset;
  float Deadzonecenter = 1475;
  float Dzoffset = -5;

  float servoMicroseconds;

  if (!invert) {
    if (output > 0) {
      servoMicroseconds = (Deadzonecenter + Dzoffset) + output * (Maxms - (Deadzonecenter + Dzoffset)) / rng;
    } else if (output < 0) {
      servoMicroseconds = Minms + (output + rng) * ((Deadzonecenter - Dzoffset) - Minms) / rng;
    } else {
      servoMicroseconds = Deadzonecenter;
    }
  } else {
    if (output > 0) {
      servoMicroseconds = Minms + output * ((Deadzonecenter - Dzoffset) - Minms) / rng;
    } else if (output < 0) {
      servoMicroseconds = (Deadzonecenter + Dzoffset) + (output + rng) * (Maxms - (Deadzonecenter + Dzoffset)) / rng;
    } else {
      servoMicroseconds = Deadzonecenter;
    }
  }

  // Optional output rate limiter
  static float lastServo = Deadzonecenter;
  float delta = servoMicroseconds - lastServo;
  if (abs(delta) > maxDeltaServo) {
    delta = maxDeltaServo * (delta > 0 ? 1 : -1);
  }
  lastServo += delta;

  return lastServo;
}


// ENS220 Sensor Initialization
void SingleShotMeasure_setup() {
  // Begin I2C and wait for ENS220 to respond
  i2c_1.begin(Wire, I2C_ADDRESS);
  while (ens220.begin(&i2c_1) != true) {
    Serial.println("Waiting for I2C to start");
    delay(1000);
  }
  Serial.print("Device UID: ");
  Serial.println(ens220.getUID(), HEX);

  // --- NEW CONFIG FOR FASTER CONVERSION ---

  // 1) Choose a shorter pressure conversion time:
  //    T_2_5 = ~2.5 ms instead of T_8_2 (8.2 ms).
  ens220.setPressureConversionTime(ENS220::PressureConversionTime::T_4_1);

  // 2) Use lower oversampling on pressure and temperature:
  //    N_4 cuts the filter overhead in half; N_2 cuts it to one-quarter; N_1 is fastest.
  //    Here we pick N_4 as a good tradeoff. If you want under 10 ms total, go N_2 or N_1.
  ens220.setOversamplingOfPressure(ENS220::Oversampling::N_4);
  ens220.setOversamplingOfTemperature(ENS220::Oversampling::N_4);

  // 3) Keep the default PT ratio (PT_8 is fine), or drop it if you want even less time.
  ens220.setPressureTemperatureRatio(ENS220::PressureTemperatureRatio::PT_8);

  // 4) Still in OneShot mode (so your loop’s SingleShotMeasure_loop() can trigger each time).
  ens220.setStandbyTime(ENS220::StandbyTime::OneShotOperation);

  // 5) Use direct data path (no bypass queue).
  ens220.setPressureDataPath(ENS220::PressureDataPath::Direct);

  // Write this faster config to the sensor.
  ens220.writeConfiguration();
}


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
      pressure = ens220.getPressureHectoPascal();
      temperature = ens220.getTempCelsius();
    }
  }
}

void setup() {
  //----------------Tilt compensation----------------
  delay(50);
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(500);
  Wire.begin();
  delay(50);
  Wire.setClock(400000);

  delay(3000);  // Allow RunCam startup time

  // Initialize command buffer
  txBuf[0] = 0xCC;
  txBuf[1] = 0x01;
  txBuf[2] = 0x01;
  txBuf[3] = calcCrc(txBuf, 3);
  delay(50);
  delay(50);
  debugCheckpoint("wire set");
  if (!mag.init()) {
    Serial.println("Failed to detect and initialize LIS3MDL magnetometer!");
  }
  mag.enableDefault();

  if (!imu.init()) {
    Serial.println("Failed to detect and initialize LSM6 IMU!");
  }
  imu.enableDefault();

  Serial.println("Please rotate device for 10 seconds to calibrate...");
  //----------------end tilt compensation settup---------
  debugCheckpoint("sensor init 1");
  camServo.attach(SERVO_PIN);
  debugCheckpoint("servo");

  // Initialize cameras control pins (e.g., for power on/off)
  pinMode(PULSECOMS, INPUT_PULLUP);
  pinMode(feedbackPin, INPUT);
  //attachInterrupt(digitalPinToInterrupt(PULSECOMS), readPulseComs, RISING);  // Trigger on HIGH pulse
  //debugCheckpoint("interupt pin enable");

  debugCheckpoint("camera stuff");

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
  } else {
    Serial.println("SD card initialized successfully");
  }
  dataFile = SD.open("data.txt", FILE_WRITE);

  debugCheckpoint("sd");

  if (!lis3mdl.begin_I2C(MAG1_I2C_ADDRESS)) {
    Serial.println("Failed to find LIS3MDL #1");
  } else {
    Serial.println("LIS3MDL #1 Found!");
    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  }

  // Initialize first IMU (LSM6DS3)
  if (!IMU.begin()) {
    Serial.println("Error initializing LSM6DS3 #1!");
  } else {
    Serial.print("Accelerometer #1 sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.println("Acceleration in g's");
    Serial.println("X\tY\tZ");
  }

  // Initialize LIS3MDL (Magnetometer)
  if (!lis3mdl.begin_I2C()) {  // hardware I2C mode
    Serial.println("Failed to find LIS3MDL chip");
  } else {
    Serial.println("LIS3MDL Found!");
    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  }
  debugCheckpoint("round two sensor init");

  SingleShotMeasure_setup();

  delay(5000);
  static unsigned long startTime = millis();
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
}
int k = 0;
void loop() {
  for (k = 0; k < 100; k++) {
    if (true) {
      Serial.print(millis());
      Serial.println("Telem loopstart");
      // Calibration phase
      int pinState = digitalRead(PULSECOMS);

      // Start recording when pin goes LOW and not already recording
      if (pinState == LOW && recState == 0) {
        Serial1.write(txBuf, 4);
        recState = 1;
        Serial.println("Started Recording");
      }

      // Wait for pin to be HIGH for 1 second before stopping recording
      if (pinState == HIGH && recState == 1) {
        if (!wasHigh) {
          highStartTime = millis();
          wasHigh = true;
        } else if (millis() - highStartTime >= 15000) {
          Serial1.write(txBuf, 4);
          recState = 0;
          Serial.println("Stopped Recording");
          wasHigh = false;
        }
      } else if (pinState == LOW) {
        wasHigh = false;  // Reset if pin goes LOW again during the 1s countdown
      }

      mag.read();
      imu.read();
      // Read magnetometer data
      sensors_event_t magEvent;
      lis3mdl.getEvent(&magEvent);
      magX = magEvent.magnetic.x;
      magY = magEvent.magnetic.y;
      magZ = magEvent.magnetic.z;
      // Read accelerometer and gyroscope data
      float accelX, accelY, accelZ;
      float gyroX, gyroY, gyroZ;
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accelX, accelY, accelZ);
      }
      // Compute heading and update running average
      float heading = computeTiltCompensatedHeading();

      tHigh = pulseIn(feedbackPin, HIGH, timeout);
      tLow = pulseIn(feedbackPin, LOW, timeout);

      float tCycle = tHigh + tLow;
      dutyCycle = (tHigh / tCycle) * 100.0;

      // Datasheet values
      const float dutyMin = 2.9;
      const float dutyMax = 96.3;
      const float fullCircle = 360.0;

      // Angle calculation
      currentAngle = ((dutyCycle - dutyMin) * fullCircle) / (dutyMax - dutyMin + 1);

      // Clamp angle between 0 and 359.99
      if (currentAngle < 0) currentAngle = 0;
      else if (currentAngle >= 360) currentAngle = 359.99;

      camServo.writeMicroseconds(pidControl(heading, setpoint, lastError, integral, lastDerivative, true, currentAngle, 0));

      // Read gyroscope data for velocity
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyroX, gyroY, gyroZ);
      }

      SingleShotMeasure_loop();
      float altitude = calculateAltitude(pressure);

      // Update altitude and velocity history
      updateAltitudeHistory(altitudeHistory, timestampHistory, altitude, historySize);
      updateVelocityHistory(altitudeHistory, velocityHistory, timestampHistory, historySize);

      logTelemetry(altitude, temperature, pressure,
                   gyroX, gyroY, gyroZ, accelX, accelY, accelZ,
                   magX, magY, magZ, heading, lastError);  //Cam-non-staiblize telem
      // Call flushBuffer() periodically or when done
    }
  }

  for (k = 0; k < 100; k++) {
    if (true) {
      //Serial.print(millis()); 
      //Serial.println("Stab loopstart");
      // Calibration phase
      int pinState = digitalRead(PULSECOMS);

      // Start recording when pin goes LOW and not already recording
      if (pinState == LOW && recState == 0) {
        Serial1.write(txBuf, 4);
        recState = 1;
        Serial.println("Started Recording");
      }

      // Wait for pin to be HIGH for 1 second before stopping recording
      if (pinState == HIGH && recState == 1) {
        if (!wasHigh) {
          highStartTime = millis();
          wasHigh = true;
        } else if (millis() - highStartTime >= 15000) {
          Serial1.write(txBuf, 4);
          recState = 0;
          Serial.println("Stopped Recording");
          wasHigh = false;
        }
      } else if (pinState == LOW) {
        wasHigh = false;  // Reset if pin goes LOW again during the 1s countdown
      }
      //Serial.println(millis());
      mag.read();
      imu.read();
      // Read magnetometer data
      sensors_event_t magEvent;
      lis3mdl.getEvent(&magEvent);
      magX = magEvent.magnetic.x;
      magY = magEvent.magnetic.y;
      magZ = magEvent.magnetic.z;
      // Read accelerometer and gyroscope data
      float accelX, accelY, accelZ;
      float gyroX, gyroY, gyroZ;
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accelX, accelY, accelZ);
      }
      //Serial.println(millis());
      // Compute heading and update running average
      float heading = computeTiltCompensatedHeading();
      //Serial.println(millis());
      tHigh = pulseIn(feedbackPin, HIGH, timeout);
      tLow = pulseIn(feedbackPin, LOW, timeout);

      float tCycle = tHigh + tLow;
      dutyCycle = (tHigh / tCycle) * 100.0;

      // Datasheet values
      const float dutyMin = 2.9;
      const float dutyMax = 96.3;
      const float fullCircle = 360.0;

      // Angle calculation
      currentAngle = ((dutyCycle - dutyMin) * fullCircle) / (dutyMax - dutyMin + 1);

      // Clamp angle between 0 and 359.99
      if (currentAngle < 0) currentAngle = 0;
      else if (currentAngle >= 360) currentAngle = 359.99;

      //Serial.println(millis());//Serial.print("PID start");

      camServo.writeMicroseconds(pidControl(heading, setpoint, lastError, integral, lastDerivative, true, currentAngle, 0));
      
      //Serial.println(millis());

      // Read gyroscope data for velocity
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyroX, gyroY, gyroZ);
      }
      
      //Serial.println(millis());
      
      SingleShotMeasure_loop();
      float altitude = calculateAltitude(pressure);
      
      //Serial.println(millis());
      //Serial.print("// Update altitude and velocity history\nupdateAltitudeHistory(altitudeHistory, timestampHistory, altitude, historySize);\nupdateVelocityHistory(altitudeHistory, velocityHistory, timestampHistory, historySize);");
      // Update altitude and velocity history
      updateAltitudeHistory(altitudeHistory, timestampHistory, altitude, historySize);
      updateVelocityHistory(altitudeHistory, velocityHistory, timestampHistory, historySize);

      //Serial.println(millis());

      logTelemetryStab(altitude, temperature, pressure,
                       gyroX, gyroY, gyroZ, accelX, accelY, accelZ,
                       magX, magY, magZ, heading, lastError);  //Cam-non-staiblize telem
    }
  }
}




void logTelemetry(float altitude, float temperature, float pressure,
                  float gyroX, float gyroY, float gyroZ,
                  float accelX, float accelY, float accelZ,
                  float magX, float magY, float magZ, float heading, float lastError) {
  // Temporary buffer for one telemetry string
  char temp[256];
  // Format the telemetry data into temp
  int len = snprintf(temp, sizeof(temp),
                     "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,COSMOS,T\n",
                     altitude, temperature, pressure,
                     gyroX, gyroY, gyroZ, accelX, accelY, accelZ,
                     magX, magY, magZ, heading, lastError);

  // Check if adding this string would exceed the buffer size
  if (bufferIndex + len >= 1024) {
    //Serial.print(millis());
    //Serial.println("Started write");
    // Write the current buffer to the file
    File dataFile = SD.open("data.txt", FILE_WRITE);
    if (dataFile) {
      //Serial.println("writing");
      dataFile.write(buffer, bufferIndex);
      dataFile.close();
    }
    bufferIndex = 0;  // Reset the buffer
    //Serial.print(millis());
    //Serial.println("completed write");
  }

  // Append the formatted string to the buffer
  memcpy(buffer + bufferIndex, temp, len);
  bufferIndex += len;
}

void logTelemetryStab(float altitude, float temperature, float pressure,
                      float gyroX, float gyroY, float gyroZ,
                      float accelX, float accelY, float accelZ,
                      float magX, float magY, float magZ, float heading, float lastError) {
  // Temporary buffer for one telemetry string
  char temp[256];
  // Format the telemetry data into temp
  int len = snprintf(temp, sizeof(temp),
                     "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,COSMOS,S\n",
                     altitude, temperature, pressure,
                     gyroX, gyroY, gyroZ, accelX, accelY, accelZ,
                     magX, magY, magZ, heading, lastError);

  // Check if adding this string would exceed the buffer size
  if (bufferIndex + len >= 1024) {
    //Serial.print(millis());
    //Serial.println("Started write");
    // Write the current buffer to the file
    File dataFile = SD.open("data.txt", FILE_WRITE);
    if (dataFile) {
      //Serial.println("writing");
      dataFile.write(buffer, bufferIndex);
      dataFile.close();
    }
    bufferIndex = 0;  // Reset the buffer
    //Serial.print(millis());
    //Serial.println("completed write");
  }

  // Append the formatted string to the buffer
  memcpy(buffer + bufferIndex, temp, len);
  bufferIndex += len;
}

void flushBuffer() {
  // Write any remaining data in the buffer
  if (bufferIndex > 0) {
    File dataFile = SD.open("data.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.write(buffer, bufferIndex);
      dataFile.close();
    }
    bufferIndex = 0;
  }
}


//__________-_-_-___  PulseComs™ by Caleb Wiley __________-_-_-_-_-_______________________
