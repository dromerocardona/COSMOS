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
#include "FeedBackServo.h"
#include <Servo.h>




const int altitude = 0;

const int gyroX = 0;
const int gyroY = 0;
const int gyroZ = 0;
const int accelX = 0;
const int accelY = 0;
const int accelZ = 0;





//----------------Tilt compensation----------------
#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>

LIS3MDL mag;
LSM6 imu;

/*
Calibration values; the default values of +/-32767 for each axis
lead to an assumed magnetometer bias of 0. Use the Calibrate example
program to determine appropriate values for your particular unit.
*/
LIS3MDL::vector<int16_t> m_min = { -32767, -32767, -32767 };
LIS3MDL::vector<int16_t> m_max = { +32767, +32767, +32767 };


/*
Returns the angular difference in the horizontal plane between the
"from" vector and north, in degrees.

Description of heading algorithm:
Shift and scale the magnetic reading based on calibration data to find
the North vector. Use the acceleration readings to determine the Up
vector (gravity is measured as an upward acceleration). The cross
product of North and Up vectors is East. The vectors East and North
form a basis for the horizontal plane. The From vector is projected
into the horizontal plane and the angle between the projected vector
and horizontal north is returned.
*/
#include <cmath>  // For atan2, fmod, and M_PI

float computeHeading() {
    // Magnetometer readings
    LIS3MDL::vector<int32_t> temp_m = { mag.m.x, mag.m.y, mag.m.z };

    // Acceleration vector with corrected orientation
    LIS3MDL::vector<int16_t> a = { -imu.a.y, imu.a.x, imu.a.z };

    // Subtract offsets from magnetometer readings
    temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
    temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;
    temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2;

    // Compute East and North vectors
    LIS3MDL::vector<float> E;
    LIS3MDL::vector<float> N;
    LIS3MDL::vector_cross(&temp_m, &a, &E);
    LIS3MDL::vector_normalize(&E);
    LIS3MDL::vector_cross(&a, &E, &N);
    LIS3MDL::vector_normalize(&N);

    // Compute angle in YZ-plane from N to Z-axis
    float theta_N_rad = atan2(N.z, N.y);                // Angle of N from Y-axis (radians)
    float theta_N_deg = theta_N_rad * (180.0f / M_PI); // Convert to degrees
    float cross = N.y * E.z - N.z * E.y;                // 2D cross product in YZ-plane
    float angle;
    if (cross > 0) {
        angle = 90.0f - theta_N_deg; // Counterclockwise from N to Z
    } else {
        angle = theta_N_deg - 90.0f; // Clockwise from N to Z
    }
    // Ensure angle is in [0, 360)
    angle = fmod(angle + 360.0f, 360.0f);
    return angle;
}

/*
Returns the angular difference in the horizontal plane between a
default vector (the +X axis) and north, in degrees.
*/









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
#define SD_CS_PIN 6  // Chip select pin for SD card

#define SERVO_PIN A5  // Servo pin for GND camera stabilization
//#define FEEDBACK_PIN 9           // Feedback signal pin for servo control
#define I2C_ADDRESS 0x20         // I2C Address for ENS 220
#define MAG1_I2C_ADDRESS 0x1C    // First LIS3MDL address
// This pin is not connected to anything on the PID board #define CAMERA2_PIN 11           // Ground camera
#define TEAM_ID "3195"

#define PULSECOMS A4

ScioSense::ENS220 ens220;
I2cInterface i2c_1;            // Added for ENS220 single-shot mode
Adafruit_LIS3MDL lis3mdl;      // Magnetometer
Adafruit_LIS3MDL lis3mdl_FC;   // First magnetometer
Adafruit_LIS3MDL lis3mdl_CAM;  // Second magnetometer
Servo servo;
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

// Camera stabilization variables
float cameraposition = 0;
unsigned long lastRpmTime = 0;        // last time of an magnet detection
volatile unsigned long rpmCount = 0;  // RPM counter
float currentInterruptTime = 0;       // Current time of an interrupt
float timeDifference = 0;             // Time difference between two consecutive interrupts
float lastInterruptTime = 0;

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
float setpoint = 180.0;           // Desired setpoint placeholder
float input = 0.0;                 // Current system input
float output = 0.0;                // PID output
float error = 0.0;                 // Current error
float lastError = 0.0;             // Previous error
float integral = 0.0;              // tracks cumulative error
const float K_proportional = 1.0;  // Proportional gain
const float K_integral = 0.1;      // Integral gain
const float K_derivative = 0.01;   // Derivative gain
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

  // Calculate the latest velocity
  latestVelocity = (altitudeHistory[0] - altitudeHistory[1]) / timeDifferenceSeconds;

  // Shift all elements in velocityHistory to the right
  for (int i = size - 1; i > 0; i--) {
    velocityHistory[i] = velocityHistory[i - 1];
  }

  // Update the first element with the latest velocity
  velocityHistory[0] = latestVelocity;
}

float avg(float arr[], int size) {
  float sum = 0.0;
  for (int i = 0; i < size; i++) {
    sum += arr[i];
  }
  return sum / size;
}

// Function for PID Camera Stabilization
String pidControl(float input, float setpoint, float &lastError, float &integral, Servo &camServo) {
  // PID tuning parameters
  const float K_proportional = 1.0;  // Proportional gain
  const float K_integral = 0.1;      // Integral gain
  const float K_derivative = 0.01;   // Derivative gain

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

  // Map heading to servo angle (assuming servo range is 0-180°)
  int currentAngle = camServo.read();              // Read current servo position
  int targetAngle = map(input, 0, 360, 0, 180);    // Adjust based on servo range
  int adjustedAngle = currentAngle - (int)output;  // Adjusting servo based on the PID output

  // Constrain to servo range (0-180°)
  if (adjustedAngle < 0) adjustedAngle = 0;
  if (adjustedAngle > 180) adjustedAngle = 180;

  // Convert target angle to microseconds (1000 to 2000 range)
  int targetMicroseconds = map(targetAngle, 0, 180, 1000, 2000);
  camServo.writeMicroseconds(targetMicroseconds);  // Set servo to target microseconds

  // Read feedback and correct (optional, depends on FeedBackServo implementation)
  if (abs(currentAngle - targetAngle) > 5) {         // Tolerance of 5 degrees
    camServo.writeMicroseconds(targetMicroseconds);  // Reapply correction
  }
  /*
  // Debug output
  Serial.print("Heading: ");
  Serial.print(input);
  Serial.print("°  Target Servo Angle: ");
  Serial.print(targetAngle);
  Serial.print("°  Current Servo Angle: ");
  Serial.println(currentAngle);
  */
  String telem = String(input, 1) + "," + String((float)targetAngle, 1) + "," + String((float)currentAngle, 1);
  return telem;  // Return the String object
}

void rpmISR() {
  currentInterruptTime = millis();                            // Get the current time
  timeDifference = currentInterruptTime - lastInterruptTime;  // Calculate the time difference between interrupts
  lastInterruptTime = currentInterruptTime;
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
  debugCheckpoint("wire set");
  if (!mag.init()) {
    Serial.println("Failed to detect and initialize LIS3MDL magnetometer!");
  }
  mag.enableDefault();

  if (!imu.init()) {
    Serial.println("Failed to detect and initialize LSM6 IMU!");
  }
  imu.enableDefault();
  //----------------end tilt compensation settup---------
  debugCheckpoint("sensor init 1");

  /*--removed for non-functionality
  // Initialize camera servo
  camServo.attach(SERVO_PIN);
  */

  debugCheckpoint("servo");

  // Initialize cameras control pins (e.g., for power on/off)
  pinMode(PULSECOMS, INPUT);
  attachInterrupt(digitalPinToInterrupt(PULSECOMS), readPulseComs, RISING);  // Trigger on HIGH pulse
  debugCheckpoint("interupt pin enable");

  pinMode(CAMERA2_PIN, OUTPUT);
  digitalWrite(CAMERA2_PIN, LOW);

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
}

void loop() {



  //----------------Tilt compensation----------------
  mag.read();
  imu.read();

  /*
  When given no arguments, the heading() function returns the angular
  difference in the horizontal plane between a default vector (the
  +X axis) and north, in degrees.
  */
  debugCheckpoint("pre-pid");
  float heading = computeHeading();
  debugCheckpoint("heading calc");
  Serial.println(pidControl(heading, setpoint, lastError, integral, camServo));
  debugCheckpoint("pid");
  //Serial.println(heading,setpoint)

  /*
  To use a different vector as a reference, use the version of
  computeHeading() that takes a vector argument; for example, call it like this
  to use the -Z axis as a reference:
  */
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
}

//__________-_-_-___  PulseComs™ by Caleb Wiley __________-_-_-_-_-_______________________
