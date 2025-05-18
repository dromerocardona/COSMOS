// LIBRARY INCLUSIONS
#include <Arduino_LSM6DS3.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <ScioSense_ENS220.h>
#include <SPI.h>
#include <SD.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include <RTClib.h>

// PINS AND DEFINITIONS
#define BATTERY_PIN A0           // Analog pin for voltage divider circuit
#define RPM_PIN A4               // Pin for Hall effect sensor
#define SD_CS_PIN 6              // Chip select pin for SD card
#define RELEASE_PIN 12           // Release servo pin
#define SERVO_PIN 13             // Servo pin for GND camera stabilization
#define FEEDBACK_PIN 9           // Feedback signal pin for servo control
#define I2C_ADDRESS 0x20         // I2C Address for ENS 220
#define DS1307_I2C_ADDRESS 0x68  // I2C Address for DS1307
#define SERIAL_BAUDRATE 57600    // Speed of Serial Communication with the computer (ENS220)
#define INTN_1 4                 // Interrupt pin for ENS220
#define CAMERA1_PIN 10           // Blade camera
#define CAMERA2_PIN 11           // Ground camera
#define LED_DATA 5
#define NUM_LEDS 5             // Number of LEDs for FastLED
#define MAG1_I2C_ADDRESS 0x1C  // First LIS3MDL address
#define MAG2_I2C_ADDRESS 0x1E  // Second LIS3MDL address
#define IMU1_I2C_ADDRESS 0x6A  // First LSM6DS3 address
#define IMU2_I2C_ADDRESS 0x6B  // Second LSM6DS3 address
#define TEAM_ID "3195"

// STATE MANAGEMENT VARIABLES
enum FlightState {
  LAUNCH_PAD,
  ASCENT,
  APOGEE,
  DESCENT,
  LANDED
};

// Objects
FlightState flightState = LAUNCH_PAD;  // Initial state
Adafruit_NeoPixel pixels(NUM_LEDS, LED_DATA, NEO_GRB + NEO_KHZ800);
ENS220 ens220;
Adafruit_LIS3MDL lis3mdl;      // Magnetometer
Adafruit_LIS3MDL lis3mdl_FC;   // First magnetometer
Adafruit_LIS3MDL lis3mdl_CAM;  // Second magnetometer
Servo servo;
Servo camServo;
File dataFile;
File backupFile;
SFE_UBLOX_GNSS gps;
RTC_DS1307 rtc;      // DS1307

// Variables
unsigned long landedTime = 0;
unsigned long lastOrientationTime = 0;
float lastOrientationX = 0.0, lastOrientationY = 0.0, lastOrientationZ = 0.0;
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
float setpoint = 1000.0;           // Desired setpoint placeholder
float input = 0.0;                 // Current system input
float output = 0.0;                // PID output
float error = 0.0;                 // Current error
float lastError = 0.0;             // Previous error
float integral = 0.0;              // tracks cumulative error
const float K_proportional = 1.0;  // Proportional gain
const float K_integral = 0.1;      // Integral gain
const float K_derivative = 0.01;   // Derivative gain
float heading;

// Calibration biases for gyroscope and accelerometer
float gyroBiasX = 0.0, gyroBiasY = 0.0, gyroBiasZ = 0.0;
float accelBiasX = 0.0, accelBiasY = 0.0, accelBiasZ = 0.0;

// Variables to store GPS data
double latitude = 0.0;
double longitude = 0.0;
double gpsAltitude = 0.0;

///////////////////////// FUNCTIONS /////////////////////////

// Function definition (add this below other functions)
void activateReleaseMechanism() {
  servo.writeMicroseconds(500);  // Activate servo to 90 degrees for release
  Serial.println("Release mechanism activated - Servo set to 90 degrees");
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

void calibrateGyroscope() {
  const int numSamples = 100;
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;

  Serial.println("Calibrating gyroscope...");
  for (int i = 0; i < numSamples; i++) {
    if (IMU.gyroscopeAvailable()) {
      float gx, gy, gz;
      IMU.readGyroscope(gx, gy, gz);
      sumX += gx;
      sumY += gy;
      sumZ += gz;
    }
    delay(10); // Small delay between samples
  }

  gyroBiasX = sumX / numSamples;
  gyroBiasY = sumY / numSamples;
  gyroBiasZ = sumZ / numSamples;

  Serial.print("Gyroscope bias: ");
  Serial.print(gyroBiasX);
  Serial.print(", ");
  Serial.print(gyroBiasY);
  Serial.print(", ");
  Serial.println(gyroBiasZ);
}

void calibrateAccelerometer() {
  const int numSamples = 100;
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;

  Serial.println("Calibrating accelerometer...");
  for (int i = 0; i < numSamples; i++) {
    if (IMU.accelerationAvailable()) {
      float ax, ay, az;
      IMU.readAcceleration(ax, ay, az);
      sumX += ax;
      sumY += ay;
      sumZ += az;
    }
    delay(10); // Small delay between samples
  }

  accelBiasX = sumX / numSamples;
  accelBiasY = sumY / numSamples;
  accelBiasZ = (sumZ / numSamples) - 9.81; // Subtract gravity (1g)

  Serial.print("Accelerometer bias: ");
  Serial.print(accelBiasX);
  Serial.print(", ");
  Serial.print(accelBiasY);
  Serial.print(", ");
  Serial.println(accelBiasZ);
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
void pidControl(float input, float setpoint, float &lastError, float &integral, Servo &camServo) {
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
  int currentAngle = camServo.read();                 // Read current servo position
  int targetAngle = map(input, 0, 360, 0, 180);    // Adjust based on servo range
  int adjustedAngle = currentAngle - (int)output;  // Adjusting servo based on the PID output

  // Constrain to servo range (0-180°)
  if (adjustedAngle < 0) adjustedAngle = 0;
  if (adjustedAngle > 180) adjustedAngle = 180;

  // Convert target angle to microseconds (1000 to 2000 range)
  int targetMicroseconds = map(targetAngle, 0, 180, 1000, 2000);
  camServo.writeMicroseconds(targetMicroseconds);  // Set servo to target microseconds

  // Read feedback and correct (optional, depends on FeedBackServo implementation)
  if (abs(currentAngle - targetAngle) > 5) {  // Tolerance of 5 degrees
    camServo.writeMicroseconds(targetMicroseconds);  // Reapply correction
  }

  // Debug output
  Serial.print("Heading: ");
  Serial.print(input);
  Serial.print("°  Target Servo Angle: ");
  Serial.print(targetAngle);
  Serial.print("°  Current Servo Angle: ");
  Serial.println(currentAngle);
}

void rpmISR() {
  currentInterruptTime = millis();                            // Get the current time
  timeDifference = currentInterruptTime - lastInterruptTime;  // Calculate the time difference between interrupts
  lastInterruptTime = currentInterruptTime;
}

void handleCommand(const char *command) {

  Serial.println(command);

  char field1[10], field2[10], field3[10], field4[10];
  int num = sscanf(command, "CMD, %*d, %9[^,], %9[^,], %9[^,], %9[^,]",
                   field1, field2, field3, field4);
  if (num < 1) {
    Serial.println("Invalid command");
    return;
  }

  float tempPressure;

  // Create a command identifier for switch
  int cmdType = 0;
  if (strcmp(field1, "CX") == 0) cmdType = 1;
  else if (strcmp(field1, "ST") == 0) cmdType = 2;
  else if (strcmp(field1, "SIM") == 0) cmdType = 3;
  else if (strcmp(field1, "SIMP") == 0) cmdType = 4;
  else if (strcmp(field1, "CAL") == 0) cmdType = 5;
  else if (strcmp(field1, "MEC") == 0) cmdType = 6;
  else if (strcmp(field1, "PARTY") == 0) cmdType = 7;

  switch (cmdType) {
    case 1:  // CX commands
      if (strcmp(field2, "ON") == 0) {
        telemetryEnabled = true;
        strncpy(lastCommand, "CXON", sizeof(lastCommand));
        Serial.println("Telemetry started.");
      } else if (strcmp(field2, "OFF") == 0) {
        telemetryEnabled = false;
        strncpy(lastCommand, "CXOFF", sizeof(lastCommand));
        Serial.println("Telemetry stopped.");
      }
      break;

    case 2:  // ST commands
      if (strcmp(field2, "UTC_TIME") == 0) {
        Serial.println("ST UTC_TIME command received.");
        strncpy(currentTime, field3, sizeof(currentTime));
        strncpy(lastCommand, "ST_UTC_TIME", sizeof(lastCommand));
      } else if (strcmp(field2, "GPS") == 0) {
        Serial.println("ST GPS command received.");
        strncpy(currentTime, gpsTime, sizeof(currentTime));
        strncpy(lastCommand, "ST_GPS", sizeof(lastCommand));
      }
      break;

    case 3:  // SIM commands
      if (strcmp(field2, "ENABLE") == 0) {
        simulationMode = true;
        Serial.println("Simulation mode enabled.");
        strncpy(lastCommand, "SIM_ENABLE", sizeof(lastCommand));
      } else if (strcmp(field2, "ACTIVATE") == 0 && simulationMode) {
        Serial.println("Simulation activated. Waiting for pressure input...");
        strncpy(lastCommand, "SIM_ACTIVATE", sizeof(lastCommand));
        char pressureInput[16];
        if (Serial1.available()) {
          Serial1.readBytesUntil('\n', pressureInput, sizeof(pressureInput));
          receivedPressure = atof(pressureInput);
        if (receivedPressure > 0.0) {
          simulatedPressure = receivedPressure;
          Serial.println("Simulated pressure updated.");
        } else {
          Serial.println("Invalid pressure value received.");
        }
      } else if (strcmp(field2, "DISABLE") == 0) {
        simulationMode = false;
        Serial.println("Simulation mode disabled.");
        strncpy(lastCommand, "SIM_DISABLE", sizeof(lastCommand));
      }
      break;

    case 4:  // SIMP command
      tempPressure = atof(field2);
      if (tempPressure > 0.0) {
        simulatedPressure = tempPressure;
        Serial.println("Simulated pressure set via SIMP.");
        strncpy(lastCommand, "SIMP", sizeof(lastCommand));
      } else {
        Serial.println("Invalid pressure value in SIMP command.");
      }
      break;

    case 5:  // CAL command
      Serial.println("CAL command received.");
      strncpy(lastCommand, "CAL", sizeof(lastCommand));
      ens220.singleShotMeasure(ENS220_SENSOR_TEMPERATURE_AND_PRESSURE);
      ens220.waitSingleShot();
      if(ens220.hasDataStatusFlag(ENS220_DATA_STATUS_PRESSURE_READY) && ens220.hasDataStatusFlag(ENS220_DATA_STATUS_TEMPERATURE_READY)) {
        referencePressure = ens220.getPressureHectoPascal();
        Serial.print("Calibration complete. Reference pressure set to: ");
        Serial.println(referencePressure);
      } else {
        Serial.println("Failed to calibrate pressure.");
      }
      calibrateGyroscope();  // Call calibration function for gyroscope
      calibrateAccelerometer();  // Call calibration function for accelerometer
      flightState = LAUNCH_PAD;
      break;

    case 6:  // MEC commands
      if (strcmp(field2, "RELEASE") == 0) {
        if (strcmp(field3, "ON") == 0) {
          Serial.println("MEC RELEASE ON command received.");
          activateReleaseMechanism();  // Activate release mechanism on command
          strncpy(lastCommand, "MEC_RELEASE_ON", sizeof(lastCommand));
        } else if (strcmp(field3, "OFF") == 0) {
          Serial.println("MEC RELEASE OFF command received.");
          servo.writeMicroseconds(900);  // Reset servo to 0 degrees (or your "off" position)
          strncpy(lastCommand, "MEC_RELEASE_OFF", sizeof(lastCommand));
          Serial.println("Release mechanism deactivated - Servo set to 0 degrees");
        }
      } else if (strcmp(field2, "CAMERA") == 0) {
        if (strcmp(field3, "BLADE") == 0) {
          if (strcmp(field4, "ON") == 0) {
            digitalWrite(CAMERA1_PIN, HIGH);
            Serial.println("MEC CAMERA BLADE ON - Camera powered ON.");
            strncpy(lastCommand, "BLADE_CAM_ON", sizeof(lastCommand));
          } else if (strcmp(field4, "OFF") == 0) {
            digitalWrite(CAMERA1_PIN, LOW);
            Serial.println("MEC CAMERA BLADE OFF - Camera powered OFF.");
            strncpy(lastCommand, "BLADE_CAM_OFF", sizeof(lastCommand));
          }
        } else if (strcmp(field3, "GROUND") == 0) {
          if (strcmp(field4, "ON") == 0) {
            digitalWrite(CAMERA2_PIN, HIGH);
            Serial.println("MEC CAMERA GROUND ON - Camera powered ON.");
            strncpy(lastCommand, "GROUND_CAM_ON", sizeof(lastCommand));
          } else if (strcmp(field4, "OFF") == 0) {
            digitalWrite(CAMERA2_PIN, LOW);
            Serial.println("MEC CAMERA GROUND OFF - Camera powered OFF.");
            strncpy(lastCommand, "GROUND_CAM_OFF", sizeof(lastCommand));
          }
        } else if (strcmp(field3, "STABLE") == 0) {
          Serial.println("MEC CAMERA STABLE command received.");
          strncpy(lastCommand, "CAM_STABLE", sizeof(lastCommand));
          pidControl(heading, setpoint, lastError, integral, servo);
        }
      }
      break;
    
    case 7:  // PARTY commands
      if (strcmp(field2, "ON") == 0) {
        Serial.println("PARTY ON command received.");
        // Add code to turn on party mode (e.g., LED effects)
      } else if (strcmp(field2, "OFF") == 0) {
        Serial.println("PARTY OFF command received.");
        // Add code to turn off party mode (e.g., LED effects)
      }
      break;

    default:
      Serial.println("Unknown command type or transmission error");
      break;

  }
}

// ENS220 Sensor Initialization
void SingleShotMeasure_setup() {
  Serial.println("Initializing ENS220...");
  Wire.begin();
  ens220.begin(&Wire, I2C_ADDRESS); 
  Serial.print("Device UID: ");
  Serial.println(ens220.getUID(), HEX);
  
  // Configure ENS220 for single-shot measurement
  Serial.print("Device UID: "); Serial.println(ens220.getUID(), HEX);

    // Choose the desired configuration of the sensor. In this example we will use the Lowest Noise settings from the datasheet
    ens220.setDefaultConfiguration();
    // Set the Pressure ADC conversion time (MEAS_CFG register, field P_CONV)
    ens220.setPressureConversionTime(ENS220_PRESSURE_CONVERSION_TIME_T_16_4);
    // Set the Oversampling of pressure measurements (OVS_CFG register, field OVSP)
    ens220.setOversamplingOfPressure(ENS220_OVERSAMPLING_N_128);
    // Set the Oversampling of temperature measurements (OVS_CFG register, field OVST)
    ens220.setOversamplingOfTemperature(ENS220_OVERSAMPLING_N_128);
    // Set the ratio between P and T measurements as produced by the measurement engine (MEAS_CFG register, field PT_RATE)
    ens220.setPressureTemperatureRatio(ENS220_PRESSURE_TEMPERATURE_RATIO_PT_1);
    // Set the operation to One shot (STBY_CFG register, field STBY_T)
    ens220.setStandbyTime(ENS220_STANDBY_TIME_ONE_SHOT_OPERATION);
    // Set whether to use the FIFO buffer, a moving average, or none (MODE_CFG register, field FIFO_MODE)
    ens220.setPressureDataPath(ENS220_PRESSURE_DATA_PATH_DIRECT);

    // Write the desired configuration into the sensor
    ens220.writeConfiguration();
  Serial.println("ENS220 initialized successfully.");
}

void SingleShotMeasure_loop()
{
     if (ens220.singleShotMeasure(ENS220_SENSOR_TEMPERATURE_AND_PRESSURE)) {
    ens220.waitSingleShot();
    auto result = ens220.update();
    if (result == RESULT_OK) {
      if (ens220.hasDataStatusFlag(ENS220_DATA_STATUS_PRESSURE_READY) && ens220.hasDataStatusFlag(ENS220_DATA_STATUS_TEMPERATURE_READY)) {
        pressure = ens220.getPressureHectoPascal();
        temperature = ens220.getTempCelsius();
        Serial.print("P[hPa]: ");
        Serial.print(pressure);
        Serial.print("\tT[C]: ");
        Serial.println(temperature);
      }
    } else {
      Serial.println("Error reading ENS220 data.");
    }
  } else {
    Serial.println("Error starting ENS220 measurement.");
  }
}

// Updated updateTime function to use DS1307 RTC
void updateTime(char *currentTime, size_t size) {
  if (rtc.isrunning()) {
    DateTime now = rtc.now();
    snprintf(currentTime, size, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  } else {
    Serial.println("RTC not running, using fallback time.");
    // Fallback to manual increment if RTC fails
    static unsigned long previousMillis = 0;
    const unsigned long interval = 1000;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      int hours, minutes, seconds;
      if (sscanf(currentTime, "%2d:%2d:%2d", &hours, &minutes, &seconds) == 3) {
        seconds++;
        if (seconds >= 60) {
          seconds = 0;
          minutes++;
          if (minutes >= 60) {
            minutes = 0;
            hours++;
            if (hours >= 24) hours = 0;
          }
        }
        snprintf(currentTime, size, "%02d:%02d:%02d", hours, minutes, seconds);
      }
    }
  }
}

void updateFlightState(float altitude, float velocity, float x, float y, float z) {
  switch (flightState) {
    case LAUNCH_PAD:
      if (altitude > 5 && velocity > 8) {
        flightState = ASCENT;
        Serial.println("Flight state: ASCENT");
      }
      break;
    case ASCENT:
      if (velocity <= -1) {
        flightState = APOGEE;
        apogeeAltitude = altitude;
        Serial.println("Flight state: APOGEE");
      }
      break;
    case APOGEE:
      if (velocity < 0) {
        flightState = DESCENT;
        Serial.println("Flight state: DESCENT");
      }
      break;
    case DESCENT:
      if (velocity == 0 && millis() - lastOrientationTime > 10000) {
        flightState = LANDED;
        landedTime = millis();
        Serial.println("Flight state: LANDED");
      }
      break;
    case LANDED:
      {
      }
      break;
  }
  // Update last orientation values
  lastOrientationTime = millis();
  lastOrientationX = x;
  lastOrientationY = y;
  lastOrientationZ = z;
}

// Callback: updateGPSdata will be called when new NAV PVT data arrives
void updateGPSdata(UBX_NAV_PVT_data_t *ubxDataStruct)
{
    latitude = ubxDataStruct->lat / 10000000.0; // Convert to degrees
    longitude = ubxDataStruct->lon / 10000000.0; // Convert to degrees
    gpsAltitude = ubxDataStruct->hMSL / 1000.0; // Convert to meters
    satellites = ubxDataStruct->numSV;
    snprintf(gpsTime, sizeof(gpsTime), "%02d:%02d:%02d", ubxDataStruct->hour, ubxDataStruct->min, ubxDataStruct->sec);
}

void setup() {
  /*pixels.begin();
  pixels.setPixelColor(0, 255, 0, 255);
  pixels.setPixelColor(1, 0, 255, 255);
  pixels.setPixelColor(2, 255, 255, 0);
  pixels.setPixelColor(3, 0, 255, 255);
  pixels.setPixelColor(4, 255, 0, 255);
  pixels.show(); comment out for now due to hardware issues*/

  // NeoPixel initialization
  pixels.setPixelColor(0, 255, 0, 0);  // Set first pixel to red
  pixels.show();                       // Update the pixels

  Serial.begin(2000000);  // Debugging output
  Serial1.begin(9600);    // XBee communication
  Serial1.setTimeout(100);  // Set timeout to 100 ms
  Wire.begin();
  Wire.setClock(400000);
  
  // Initialize release servo
  servo.attach(RELEASE_PIN);
  // Initialize camera servo
  camServo.attach(SERVO_PIN);

  // Initialize DS1307 RTC
  if (!rtc.begin()) {
    Serial.println("Couldn't find DS1307 RTC!");
  }
  
  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running, setting time to compile time.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  // Set to compile time
  }

  // Initialize cameras control pins (e.g., for power on/off)
  pinMode(CAMERA1_PIN, OUTPUT);    // Set camera control pin to output
  digitalWrite(CAMERA1_PIN, LOW);  // Make sure camera is OFF initially
  pinMode(CAMERA2_PIN, OUTPUT);
  digitalWrite(CAMERA2_PIN, LOW);

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
  } else {
    Serial.println("SD card initialized successfully");
  }
  backupFile = SD.open("backup.txt", FILE_WRITE);

  // Initialize RPM sensor
  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmISR, RISING);

  if (!gps.begin()) {
    Serial.println("GNSS v3 initialization failed!");
  }
  gps.setI2COutput(COM_TYPE_UBX); // Set I2C output to UBX
  gps.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); // Save I2C output configuration
  gps.setNavigationFrequency(1); // Set rate of gps
  gps.setAutoPVTcallbackPtr(&updateGPSdata); // Enable automatic NAV PVT callback

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

  SingleShotMeasure_setup();
}

void loop() {

  updateTime(currentTime, sizeof(currentTime));
  // Read data from XBee or Serial1
  char command[64] = {0};
  if (Serial1.readBytesUntil('\n', command, sizeof(command) - 1) > 0) {
    command[sizeof(command) - 1] = '\0';  // Ensure null-termination
    handleCommand(command);               // Process the command
    memset(command, 0, sizeof(command));  // Clear the command buffer
  }

 // Clear remaining Serial1 buffer data
  if (Serial1.available()) {
    char discard[64];
    Serial1.readBytes(discard, min(Serial1.available(), sizeof(discard)));
    Serial.println("Discarded remaining Serial1 data");
  }

  unsigned long missionTime = millis() / 1000;  // Mission time in seconds

  // Calculate instantaneous RPM
  float rpm = (timeDifference > 0) ? (60000 / timeDifference) : 0;  // Calculate RPM
  lastRpmTime = millis();                                           // Immediately update last RPM time for min error
  rpmCount = 0;

  // Read battery voltage
  float currentVoltage = analogRead(BATTERY_PIN) * voltageDividerFactor;

  // GPS
  gps.checkUblox(); // Check for the arrival of new data and process it
  gps.checkCallbacks(); // Check if any callbacks are waiting to be processed
  Serial.print("Lat: "); Serial.print(latitude, 7);
  Serial.print(" Lon: "); Serial.print(longitude, 7);
  Serial.print(" Alt: "); Serial.print(gpsAltitude);
  Serial.print(" Sat: "); Serial.print(satellites);
  Serial.print(" Time: "); Serial.println(gpsTime);

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
    accelX -= accelBiasX;
    accelY -= accelBiasY;
    accelZ -= accelBiasZ;
    Serial.print(accelX);
    Serial.print('\t');
    Serial.print(accelY);
    Serial.print('\t');
    Serial.println(accelZ);
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    gyroX -= gyroBiasX;
    gyroY -= gyroBiasY;
    gyroZ -= gyroBiasZ;
  }
  
  // Retrieve Temperature and Pressure from ENS220 (Single Shot Mode)
  SingleShotMeasure_loop();

  // Calculate altitude from pressure
  float altitude = calculateAltitude(pressure);

  // Use simulated pressure if in simulation mode
  if (simulationMode) {
    float simulatedAltitude = (1 - pow(simulatedPressure / 1013.25, 0.190284)) * 145366.45;  // Approximation formula
    altitude = simulatedAltitude;
  }
  
  // Update maxAltitude during ascent
  if (flightState == ASCENT && altitude > maxAltitude) {
    maxAltitude = altitude;
  }
  // Telemetry Transmission
  if (lastTransmissionTime + 1000 < millis()) {  // Transmit telemetry every second
    lastTransmissionTime = millis();
    if (telemetryEnabled) {
      char telemetry[256];
      const char *mode = simulationMode ? "S" : "F";
      const char *state;
      switch (flightState) {
        case LAUNCH_PAD: state = "LAUNCH_PAD"; break;
        case ASCENT: state = "ASCENT"; break;
        case APOGEE: state = "APOGEE"; break;
        case DESCENT: state = "DESCENT"; break;
        case LANDED: state = "LANDED"; break;
        default: state = "UNKNOWN"; break;
      }
      
      snprintf(telemetry, sizeof(telemetry),
               "%s,%s,%u,%s,%s,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%s,%.1f,%.4f,%.4f,%u,%s,COSMOS",
               TEAM_ID, currentTime, packetCount, mode, state,
               altitude, temperature, pressure, currentVoltage,
               gyroX, gyroY, gyroZ, accelX, accelY, accelZ,
               magEvent1.magnetic.x, magEvent1.magnetic.y, 
               magEvent1.magnetic.z, rpm, gpsTime, gpsAltitude,
               latitude, longitude, satellites, lastCommand);
      
      Serial1.println(telemetry);
      Serial.println(telemetry);
      // Save telemetry to SD card
      dataFile = SD.open("data.txt", FILE_WRITE);
      if (dataFile) {
        dataFile.println(telemetry);
        dataFile.close();
        Serial.println("Finished writing to SD card!");
      } else {
        Serial.println("Error writing to SD card!");
      }
      packetCount++;  // Increment packet count
    }
  }

  // Calculate heading
  heading = atan2(magEvent2.magnetic.y, magEvent2.magnetic.x);
  heading = heading * 180 / PI;     // Convert to degrees
  if (heading < 0) heading += 360;  // Ensure 0-360 range

  // Used for some state transitions
  updateAltitudeHistory(altitudeHistory, timestampHistory, altitude, historySize);
  updateVelocityHistory(altitudeHistory, velocityHistory, timestampHistory, historySize);

  Serial.println("Velocity: ");
  Serial.print(latestVelocity);

  switch (flightState) {
    case LAUNCH_PAD:
      // Code for launch state
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      break;
    case ASCENT:
      // Code for ascent state
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      break;
    case APOGEE:
      // Code for Apogee state
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      break;
    case DESCENT:
      // Code for separated state
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      if (!releaseActivated && altitude <= (apogeeAltitude * 0.75)) {
        activateReleaseMechanism();
        releaseActivated = true;  // Prevent repeated activation
      }
      // PID Camera Stabilization
      pidControl(heading, setpoint, lastError, integral, servo);
      break;
    case LANDED:
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      if (avg(velocityHistory, historySize) < 1) {
        flightState = LANDED;
      }
      break;
  }
  /* pixels.setPixelColor(0, 0, 0, 255);      // LED 0: Blue
  pixels.setPixelColor(1, 0, 255, 0);      // LED 1: Green
  pixels.setPixelColor(2, 255, 0, 0);      // LED 2: Red
  pixels.setPixelColor(3, 255, 255, 0);    // LED 3: Yellow
  pixels.setPixelColor(4, 255, 255, 255);  // LED 4: White
  pixels.show();  */
}
