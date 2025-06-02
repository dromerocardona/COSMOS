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
//#include "FeedBackServo.h"
#include <Servo.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include <cmath>

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
  mag.read();
  imu.read();

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

const int pin = A4;
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
I2cInterface i2c_1;            // Added for ENS220 single-shot mode
Adafruit_LIS3MDL lis3mdl;      // Magnetometer
Adafruit_LIS3MDL lis3mdl_FC;   // First magnetometer
Adafruit_LIS3MDL lis3mdl_CAM;  // Second magnetometer
Servo camServo;
File dataFile;
File backupFile;

// Variables
unsigned long landedTime = 0;
unsigned long lastOrientationTime = 0;
//float lastOrientationX = 0.0, lastOrientationY = 0.0, lastOrientationZ = 0.0;
uint8_t satellites = 0;
float voltageDividerFactor = 0.012089;                    // Adjust based on resistor values in voltage divider
float lastTransmissionTime = 0;                           // Last time of telemetry transmission
char currentTime[9] = "00:00:00";                         // Mission time in "HH:MM:SS"
char gpsTime[9] = "00:00:00";                             // GPS time in "HH:MM:SS"
float mag2X, mag2Y, mag2Z;                                // Magnetometer data for second sensor
float accel2X, accel2Y, accel2Z, gyro2X, gyro2Y, gyro2Z;  // IMU data for second sensor (if separate IMU object is used)
char lastCommand[32];                                     // Last received command
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
float setpoint = 359;            // Desired setpoint placeholder
float input = 0.0;                 // Current system input
float output = 0.0;                // PID output
float error = 0.0;                 // Current error
float lastError = 0.0;             // Previous error
float integral = 0.0;              // tracks cumulative error
/*const float K_proportional = 1.0;  // Proportional gain ///potentialy redundant
const float K_integral = 0.1;      // Integral gain
const float K_derivative = 0.01;   // Derivative gain*/
float heading;

// PulseComs™
const int FC_COMS = PULSECOMS;  // Interrupt pin
volatile int pulseCount = 0;    // Must be 'volatile' since used in interrupt
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
    Serial.println("Three HIGH pulses detected!");
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

// Function for PID Camera Stabilization
float pidControl(float input, float setpoint, float &lastError, float &integral, bool invert) {
  // PID tuning parameters
  const float K_proportional = 1;  // Proportional gain
  const float K_integral = 0;        // Integral gain
  const float K_derivative = 1.5;      // Derivative gain
  const float rng = 360;

  // Calculate the error
  float error = setpoint - input;  // Calculate the error based on difference between setpoint and input
  if (error > 180) {
    error = error - 360;  // account for the closest path to the setpoint being across the 0° line
  } else if (error < -180) {
    error = error + 360;  // account for the closest path to the setpoint being across the 0° line the other way
  }

  // Calculate the integral term
  integral += error;

  // Calculate the derivative term
  float derivative = error - lastError;

  // Calculate the PID output
  float output = K_proportional * error + K_integral * integral + K_derivative * derivative;

  // Store the current error as the last error to prepare for the next iteration
  lastError = error;

  //Specs:-------------------------------------------
  //Control signal: PWM,  3–5 V 50 Hz,  1280–1720 µs
  //Control signal zero-speed deadband:  1480–1520 µs (+/- 15)
  //-------------------------------------------
  int MMoffset = 10;
  int Maxms = 1595+MMoffset;
  int Minms = 1355-MMoffset;
  float servoMicroseconds;
  int Deadzonecenter = 1475;
  int Dzoffset = -10; //controls how much of the deadzone to use

  if (invert == false) {
    if (output > 0) {
      servoMicroseconds = map(output, 0, rng, Deadzonecenter+Dzoffset, Maxms);
    } else if (output < 0) {
      servoMicroseconds = map(output, -rng, 0, Minms, Deadzonecenter-Dzoffset);
    } else {
    }
  } else {
    if (output > 0) {
      servoMicroseconds = map(output, 0, rng, Minms, Deadzonecenter-Dzoffset);
    } else if (output < 0) {
      servoMicroseconds = map(output, -rng, 0, Deadzonecenter+Dzoffset, Maxms);
    } else {
    }
  }
  Serial.print(output);
  Serial.print(",");
  Serial.print(error);
  Serial.print(",");
  Serial.print(servoMicroseconds);
  Serial.print(",");




  //int currentAngle = camServo.read();              // Read current servo position

  /*// Debug output
  Serial.print("Heading: ");
  Serial.print(input);
  Serial.print("°  Target Servo Angle: ");
  Serial.print(targetAngle);
  Serial.print("°  Current Servo Angle: ");
  Serial.println(currentAngle);

  //float telem = String(input, 1) + "," + String((float)targetAngle, 1) + "," + String((float)currentAngle, 1);
  //return telem;  // Return the String object
  */
  //testing

  return servoMicroseconds;
}

// ENS220 Sensor Initialization
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

void setup() {
  //----------------Tilt compensation----------------
  Serial.begin(115200);
  delay(5000);
  Serial.println("hi");
  Wire.begin();
  Wire.setClock(400000);
  Serial1.begin(115200);  //potentialy needs to be changed

  delay(3000);  // Allow RunCam startup time

  // Initialize command buffer
  txBuf[0] = 0xCC;
  txBuf[1] = 0x01;
  txBuf[2] = 0x01;
  txBuf[3] = calcCrc(txBuf, 3);

  pinMode(pin, INPUT_PULLUP);  // Fix: INPUT_PULLUP was mistyped
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
  pinMode(PULSECOMS, INPUT);
  attachInterrupt(digitalPinToInterrupt(PULSECOMS), readPulseComs, RISING);  // Trigger on HIGH pulse
  debugCheckpoint("interupt pin enable");

  debugCheckpoint("camera stuff");

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
  } else {
    Serial.println("SD card initialized successfully");
  }
  backupFile = SD.open("backup.txt", FILE_WRITE);

  debugCheckpoint("sd");

  if (!lis3mdl_FC.begin_I2C(MAG1_I2C_ADDRESS)) {
    Serial.println("Failed to find LIS3MDL #1");
  } else {
    Serial.println("LIS3MDL #1 Found!");
    lis3mdl_FC.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl_FC.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl_FC.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl_FC.setRange(LIS3MDL_RANGE_4_GAUSS);
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

void loop() {
  //----------------Tilt compensation----------------

  mag.read();
  imu.read();


  // Compute heading and update running average
  float heading = computeTiltCompensatedHeading();
  float smoothed_heading;
  updateRunningAverage(heading, smoothed_heading);
  camServo.writeMicroseconds(pidControl(heading, setpoint, lastError, integral, true));  //float input, float setpoint, float &lastError, float &integral, bool invert

  // Output for plotting (e.g., Serial Plotter)
  Serial.print("360,0,");          // Reference lines
  Serial.print(smoothed_heading);  // Smoothed heading
  Serial.print(",");
  Serial.println(heading);  // Raw heading
  //----------------Tilt compensation end----------------

  unsigned long missionTime = millis() / 1000;  // Mission time in seconds

  // Read magnetometer data
  sensors_event_t magEvent1;
  lis3mdl.getEvent(&magEvent1);
  // Read second magnetometer
  sensors_event_t magEvent2;
  lis3mdl_CAM.getEvent(&magEvent2);
  mag2X = magEvent2.magnetic.x;
  mag2Y = magEvent2.magnetic.y;
  mag2Z = magEvent2.magnetic.z;
  // Read accelerometer and gyroscope data
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    /*
    Serial.print(accelX);
    Serial.print('\t');
    Serial.print(accelY);
    Serial.print('\t');
    Serial.println(accelZ);
    */
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
  }
  debugCheckpoint("pre-singleshot");
  // Retrieve Temperature and Pressure from ENS220 (Single Shot Mode)
  SingleShotMeasure_loop();
  debugCheckpoint("successfull singleshot");

  // Calculate altitude from pressure
  float altitude = calculateAltitude(pressure);
  char telemetry[256];  // Adjust the size if necessary
  debugCheckpoint("pre-telemetry");
  snprintf(telemetry, sizeof(telemetry),
           "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,COSMOS",
           altitude, temperature, pressure, "currentVoltage",
           gyroX, gyroY, gyroZ, accelX, accelY, accelZ,
           magEvent1.magnetic.x, magEvent1.magnetic.y,
           magEvent1.magnetic.z);
  debugCheckpoint("successfull telemetry");
  //Serial.println(telemetry);
  // Save telemetry to SD card
  dataFile = SD.open("data.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(telemetry);
    dataFile.close();
    //Serial.println("Finished writing to SD card!");
  } else {
    //Serial.println("Error writing to SD card!");
  }
  packetCount++;  // Increment packet count

  /*-------dated heading calc---------
  // Calculate heading
  heading = atan2(magEvent2.magnetic.y, magEvent2.magnetic.x);
  heading = heading * 180 / PI;     // Convert to degrees
  if (heading < 0) heading += 360;  // Ensure 0-360 range
  */

  int comsState = digitalRead(FC_COMS);

  if (comsState == HIGH) {
    //Serial.println("FC_COMS is HIGH – Communication Active!");
  } else {
    //Serial.println("FC_COMS is LOW – No Communication.");
  }

  // Used for some state transitions
  updateAltitudeHistory(altitudeHistory, timestampHistory, altitude, historySize);
  updateVelocityHistory(altitudeHistory, velocityHistory, timestampHistory, historySize);

  /*
  Serial.println("Velocity: ");
  Serial.print(latestVelocity);
  */

  // Calibration phase
  int pinState = digitalRead(pin);

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
    } else if (millis() - highStartTime >= 1000) {
      Serial1.write(txBuf, 4);
      recState = 0;
      Serial.println("Stopped Recording");
      wasHigh = false;
    }
  } else if (pinState == LOW) {
    wasHigh = false;  // Reset if pin goes LOW again during the 1s countdown
  }
}

//__________-_-_-___  PulseComs™ by Caleb Wiley __________-_-_-_-_-_______________________
