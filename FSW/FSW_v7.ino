#include <Arduino_LSM6DS3.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <ScioSense_ENS220.h>
#include <utils.h>
#include <SPI.h>
#include <SD.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Arduino.h>
#include "i2c_interface.h"
#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include <RTClib.h>

// Debugging
const bool DEBUG = false;
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

// PINS AND DEFINITIONS
#define BATTERY_PIN A2
#define RPM_PIN A4
#define SD_CS_PIN 6
#define RELEASE_PIN 13
#define ENS220_I2C_ADDRESS 0x20
#define DS1307_I2C_ADDRESS 0x68
#define MAG_I2C_ADDRESS 0x1E
#define USB_BAUDRATE 115200
#define XBEE_BAUDRATE 115200
#define GND_CAM_CTRL 11 //PID board pin
#define BLADE_CAM_CTRL 10
#define LED_DATA 5
#define NUM_LEDS 5
#define TEAM_ID "3195"
#define PULSES_PER_REVOLUTION 1 // Number of pulses per revolution of the mechanism

// STATE MANAGEMENT VARIABLES
enum FlightState {
  LAUNCH_PAD,
  ASCENT,
  APOGEE,
  DESCENT,
  PROBE_RELEASE,
  LANDED
};

// Objects
FlightState flightState = LAUNCH_PAD;
Adafruit_NeoPixel pixels(NUM_LEDS, LED_DATA, NEO_GRB + NEO_KHZ800);
ScioSense::ENS220 ens220;
I2cInterface i2c_1;
Adafruit_LIS3MDL lis3mdl_FC;
Servo releaseServo;
File dataFile;
File backupFile;
SFE_UBLOX_GNSS gps;
RTC_DS1307 rtc;

// Variables
unsigned long landedTime = 0;
unsigned long lastOrientationTime = 0;
float lastOrientationX = 0.0, lastOrientationY = 0.0, lastOrientationZ = 0.0;
uint8_t satellites = 0;
float voltageDividerFactor = 0.00304;
float lastTransmissionTime = 0;
char currentTime[9] = "00:00:00";
char gpsTime[9] = "00:00:00";
float accel2X, accel2Y, accel2Z, gyro2X, gyro2Y, gyro2Z;
char lastCommand[16] = "NONE";  // Reduced from 32 to 16
unsigned int packetCount = 0;
bool telemetryEnabled = false; // SHOULD ALWAYS BE FALSE SO THT THE TELEMETRY DOESN"T START AUTOMATICALLY AND ONLY STARTS WHEN ITS CX_ON
float cameraposition = 0;
unsigned long lastRpmTime = 0;
volatile unsigned long rpmCount = 0;
volatile float currentInterruptTime = 0;
volatile float timeDifference = 0;
volatile float lastInterruptTime = 0;
float apogeeAltitude = 0.0;
float maxAltitude = 0;
bool releaseActivated = false;
bool simulationMode = false;
float simulatedPressure = 100.0;
bool firstSimpReceived = false;
float receivedPressure = 0.0;
float referencePressure = 1013.25;
const int historySize = 10;
float pressure;
float temperature;
float altitudeHistory[historySize];
float velocityHistory[historySize];
unsigned long timestampHistory[historySize];
float latestVelocity;
float gyroBiasX = 0.0, gyroBiasY = 0.0, gyroBiasZ = 0.0;
float accelBiasX = 0.0, accelBiasY = 0.0, accelBiasZ = 0.0;
double latitude = 0.0;
double longitude = 0.0;
double gpsAltitude = 0.0;
bool ens220Initialized = false;
unsigned long ens220InitStartTime = 0;
const unsigned long ENS220_INIT_TIMEOUT = 10000;
const unsigned long ENS220_INIT_INTERVAL = 1000;
bool sdFailed = false;
unsigned long lastSdBlinkTime = 0;
bool sdLedState = false;
const unsigned long SD_BLINK_INTERVAL = 500;
bool calibratingGyro = false;
bool calibratingAccel = false;
unsigned long lastCalibrationSampleTime = 0;
const unsigned long CALIBRATION_SAMPLE_INTERVAL = 10;
const int CALIBRATION_SAMPLES = 100;
int calibrationSampleCount = 0;
float calibSumX = 0.0, calibSumY = 0.0, calibSumZ = 0.0;
bool partyMode = false;
unsigned long lastPartyUpdateTime = 0;
const unsigned long PARTY_UPDATE_INTERVAL = 50;
uint8_t partyPhase = 0;
int landedVelocities = 0;

void partyModeEffect() {
  if (!partyMode) return;
  if (millis() - lastPartyUpdateTime < PARTY_UPDATE_INTERVAL) return;
  float phase = (float)partyPhase * 0.8;
  uint8_t r = (sin(phase) + 1) * 127.5;
  uint8_t g = (sin(phase + 2.094) + 1) * 127.5;
  uint8_t b = (sin(phase + 4.188) + 1) * 127.5;
  for (int i = 1; i < NUM_LEDS; i++) {
    pixels.setPixelColor(i, r, g, b);
  }
  pixels.show();
  partyPhase++;
  lastPartyUpdateTime = millis();
}

float calculateAltitude(float pressure) {
  const float temperatureLapseRate = 0.0065;
  const float seaLevelTemperature = 288.15;
  const float gasConstant = 8.3144598;
  const float molarMass = 0.0289644;
  const float gravity = 9.80665;

  if (pressure <= 0.0 || pressure > 2000.0 || referencePressure <= 0.0) {
    Serial.print(F("Invalid pressure for altitude calculation: "));
    Serial.print(pressure);
    Serial.print(F(" hPa, Reference: "));
    Serial.println(referencePressure);
    return 0.0;
  }

  float pressureRatio = pressure / referencePressure;
  float exponent = (gasConstant * temperatureLapseRate) / (gravity * molarMass);
  float altitude = (seaLevelTemperature / temperatureLapseRate) * (1.0 - pow(pressureRatio, exponent));

  if (isnan(altitude) || isinf(altitude) || altitude > 5000.0 || altitude < -1000.0) {
    Serial.print(F("Invalid or unrealistic altitude calculated: "));
    Serial.print(altitude);
    Serial.print(F(" m, Pressure: "));
    Serial.print(pressure);
    Serial.print(F(" hPa, Reference: "));
    Serial.println(referencePressure);
    return 0.0;
  }
  return altitude;
}


void calibrateGyroscope() {
  Serial.println(F("Starting gyroscope calibration..."));
  pixels.setPixelColor(1, 255, 128, 0); // Orange for CAL
  pixels.show();
  calibratingGyro = true;
  calibrationSampleCount = 0;
  calibSumX = 0.0;
  calibSumY = 0.0;
  calibSumZ = 0.0;
  unsigned long startTime = millis();

  while (calibrationSampleCount < CALIBRATION_SAMPLES && millis() - startTime < 5000) {
    if (IMU.gyroscopeAvailable()) {
      float gx, gy, gz;
      IMU.readGyroscope(gx, gy, gz);
      calibSumX += gx;
      calibSumY += gy;
      calibSumZ += gz;
      calibrationSampleCount++;
      delay(CALIBRATION_SAMPLE_INTERVAL);  // Ensure consistent sampling
    } else {
      Serial.println(F("Gyroscope data not available, retrying..."));
      delay(10);
    }
  }

  if (calibrationSampleCount >= CALIBRATION_SAMPLES) {
    gyroBiasX = calibSumX / CALIBRATION_SAMPLES;
    gyroBiasY = calibSumY / CALIBRATION_SAMPLES;
    gyroBiasZ = calibSumZ / CALIBRATION_SAMPLES;
    Serial.print(F("Gyroscope calibration complete. Bias: "));
    Serial.print(gyroBiasX);
    Serial.print(F(", "));
    Serial.print(gyroBiasY);
    Serial.print(F(", "));
    Serial.println(gyroBiasZ);
  } else {
    Serial.println(F("Gyroscope calibration failed: insufficient samples"));
    pixels.setPixelColor(1, 255, 0, 0); // red for failed CAL
    pixels.show();
  }
  calibratingGyro = false;
}

void calibrateAccelerometer() {
  Serial.println(F("Starting accelerometer calibration..."));
  pixels.setPixelColor(1, 255, 128, 0); // Orange for CAL
  pixels.show();
  calibratingAccel = true;
  calibrationSampleCount = 0;
  calibSumX = 0.0;
  calibSumY = 0.0;
  calibSumZ = 0.0;
  unsigned long startTime = millis();

  while (calibrationSampleCount < CALIBRATION_SAMPLES && millis() - startTime < 5000) {
    if (IMU.accelerationAvailable()) {
      float ax, ay, az;
      IMU.readAcceleration(ax, ay, az);
      calibSumX += ax;
      calibSumY += ay;
      calibSumZ += az;
      calibrationSampleCount++;
      delay(CALIBRATION_SAMPLE_INTERVAL);
    } else {
      Serial.println(F("Accelerometer data not available, retrying..."));
      delay(10);
    }
  }

  if (calibrationSampleCount >= CALIBRATION_SAMPLES) {
    accelBiasX = calibSumX / CALIBRATION_SAMPLES;
    accelBiasY = calibSumY / CALIBRATION_SAMPLES;
    accelBiasZ = (calibSumZ / CALIBRATION_SAMPLES) - 9.81;
    Serial.print(F("Accelerometer calibration complete. Bias: "));
    Serial.print(accelBiasX);
    Serial.print(F(", "));
    Serial.print(accelBiasY);
    Serial.print(F(", "));
    Serial.println(accelBiasZ);
  } else {
    Serial.println(F("Accelerometer calibration failed: insufficient samples"));
    pixels.setPixelColor(1, 255, 0, 0);// red for failed Accel Cal
    pixels.show();
  }
  calibratingAccel = false;
}

void updateAltitudeHistory(float altitudeHistory[], unsigned long timestampHistory[], float newAltitude, int size) {
  if (simulationMode) {
    if (newAltitude != altitudeHistory[0]) {
      for (int i = size - 1; i > 0; i--) {
        altitudeHistory[i] = altitudeHistory[i - 1];
        timestampHistory[i] = timestampHistory[i - 1];
      }
      altitudeHistory[0] = newAltitude;
      timestampHistory[0] = millis();
    }
  } else {
      for (int i = size - 1; i > 0; i--) {
        altitudeHistory[i] = altitudeHistory[i - 1];
        timestampHistory[i] = timestampHistory[i - 1];
      }
      altitudeHistory[0] = newAltitude;
      timestampHistory[0] = millis();
  }
}

void updateVelocityHistory(float altitudeHistory[], float velocityHistory[], unsigned long timestampHistory[], int size) {
  unsigned long timeDifferenceMillis = timestampHistory[0] - timestampHistory[1];
  float timeDifferenceSeconds = timeDifferenceMillis / 1000.0;
  latestVelocity = (altitudeHistory[0] - altitudeHistory[1]) / timeDifferenceSeconds;
  for (int i = size - 1; i > 0; i--) {
    velocityHistory[i] = velocityHistory[i - 1];
  }
  velocityHistory[0] = latestVelocity;
}

float avg(float arr[], int size) {
  float sum = 0.0;
  for (int i = 0; i < size; i++) {
    sum += arr[i];
  }
  return sum / size;
}


volatile unsigned long interruptCount = 0;

void rpmISR() {
  unsigned long now = micros();
  if (now - lastInterruptTime > 50000) {
    currentInterruptTime = now;
    timeDifference = (now - lastInterruptTime) / 1000.0;
    lastInterruptTime = now;
    rpmCount++;
    if (DEBUG) {
      Serial.println(F("RPM ISR triggered"));
    }
  }
}

void handleCommand(const char *command) {
  char field1[10], field2[10], field3[10], field4[10];
  char pressureStr[16];
  int num = sscanf(command, "CMD,%*[^,],%9[^,],%9[^,],%9[^,],%9[^,]", field1, field2, field3, field4);
  if (num < 1) {
    Serial.println(F("Invalid command"));
    return;
  }
  float receivedPressure;
  int cmdType = 0;
  if (strcmp(field1, "CX") == 0) cmdType = 1;
  else if (strcmp(field1, "ST") == 0) cmdType = 2;
  else if (strcmp(field1, "SIM") == 0) cmdType = 3;
  else if (strcmp(field1, "SIMP") == 0) cmdType = 4;
  else if (strcmp(field1, "CAL") == 0) cmdType = 5;
  else if (strcmp(field1, "MEC") == 0) cmdType = 6;
  else if (strcmp(field1, "PARTY") == 0) cmdType = 7;

  switch (cmdType) {
    case 1:
      if (strcmp(field2, "ON") == 0) {
        telemetryEnabled = true;
        strncpy(lastCommand, "CXON", sizeof(lastCommand));
        lastCommand[sizeof(lastCommand) - 1] = '\0';
        Serial.println(F("Telemetry started."));
        pixels.setPixelColor(1, 0, 255, 0); // green for CX_ON
        pixels.show();
      } else if (strcmp(field2, "OFF") == 0) {
        telemetryEnabled = false;
        strncpy(lastCommand, "CXOFF", sizeof(lastCommand));
        lastCommand[sizeof(lastCommand) - 1] = '\0';
        Serial.println(F("Telemetry stopped."));
        pixels.setPixelColor(1, 128, 128, 128); // Gray for CX_OFF
        pixels.show();
      }
      break;
    case 2:
      if (strcmp(field2, "UTC_TIME") == 0) {
        Serial.println(F("ST UTC_TIME command received."));
        strncpy(currentTime, field3, sizeof(currentTime));
        strncpy(lastCommand, "ST_UTC_TIME", sizeof(lastCommand));
        lastCommand[sizeof(lastCommand) - 1] = '\0';
        pixels.setPixelColor(1, 0, 50, 255); // Blue for UTC time
        pixels.show();
      } else if (strcmp(field2, "GPS") == 0) {
        Serial.println(F("ST GPS command received."));
        strncpy(currentTime, gpsTime, sizeof(currentTime));
        strncpy(lastCommand, "ST_GPS", sizeof(lastCommand));
        lastCommand[sizeof(lastCommand) - 1] = '\0';
      }
      break;
    case 3:
      {
        if (strcmp(field2, "ENABLE") == 0) {
          Serial.println(F("Simulation mode enabled."));
          strncpy(lastCommand, "SIM_ENABLE", sizeof(lastCommand));
          lastCommand[sizeof(lastCommand) - 1] = '\0';
          pixels.setPixelColor(1, 255, 0, 255); // Light Magenta for SIM_ENABLE
          pixels.show();
        } else if (strcmp(field2, "ACTIVATE") == 0) {
          simulationMode = true;
          Serial.println(F("Simulation activated. Performing calibration..."));
          strncpy(lastCommand, "SIM_ACTIVATE", sizeof(lastCommand));
          lastCommand[sizeof(lastCommand) - 1] = '\0';
          pixels.setPixelColor(1, 255, 0, 150); // Pink for SIM_ACTIVATE
          pixels.show();

          calibrateGyroscope();
          calibrateAccelerometer();
          flightState = LAUNCH_PAD;

          // Reset history arrays and related variables
          for (int i = 0; i < historySize; i++) {
            altitudeHistory[i] = 0.0;
            velocityHistory[i] = 0.0;
            timestampHistory[i] = 0;
          }
          maxAltitude = 0.0;
          apogeeAltitude = 0.0;
          latestVelocity = 0.0;
          Serial.println(F("History arrays and altitude variables reset."));

          calibrateGyroscope();
          calibrateAccelerometer();
          flightState = LAUNCH_PAD;

          pixels.setPixelColor(1, 255, 128, 0); // Orange for CAL
          pixels.show();

        } else if (strcmp(field2, "DISABLE") == 0) {
          simulationMode = false;
          Serial.println(F("Simulation mode disabled."));
          strncpy(lastCommand, "SIM_DISABLE", sizeof(lastCommand));
          lastCommand[sizeof(lastCommand) - 1] = '\0';
          pixels.setPixelColor(1, 250, 140, 140); // white red for SIM_DISABLE
          pixels.show();
        }
        break;
      }
    case 4:
      strncpy(pressureStr, field2, sizeof(pressureStr));
      pressureStr[sizeof(pressureStr) - 1] = '\0';
      receivedPressure = atof(pressureStr) / 100.0;
      if (receivedPressure > 0.0 && receivedPressure < 2000.0) {
        simulationMode = true;
        simulatedPressure = receivedPressure;

        // Calibrate on first SIMP command
        if (!firstSimpReceived) {
          referencePressure = receivedPressure;
          Serial.print(F("First SIMP command: Reference pressure set to: "));
          Serial.print(referencePressure);
          Serial.println(F(" hPa"));

          for (int i = 0; i < historySize; i++) {
            altitudeHistory[i] = 0.0;
            velocityHistory[i] = 0.0;
            timestampHistory[i] = 0;
          }
          maxAltitude = 0.0;
          apogeeAltitude = 0.0;
          latestVelocity = 0.0;
          flightState = LAUNCH_PAD;
          Serial.println(F("History arrays and altitude variables reset."));

          calibrateGyroscope();
          calibrateAccelerometer();

          firstSimpReceived = true;
          pixels.setPixelColor(1, 255, 128, 0); // Orange for CAL on SIMP
          pixels.show();
        }

        Serial.print(F("Simulated pressure set via SIMP: "));
        Serial.print(simulatedPressure);
        Serial.println(F(" hPa"));
        strncpy(lastCommand, "SIMP", sizeof(lastCommand));
        lastCommand[sizeof(lastCommand) - 1] = '\0';
        pixels.setPixelColor(1, 90, 20, 150); // Purple for SIMP
        pixels.show();
      } else {
        Serial.println(F("Invalid pressure value in SIMP command."));
        pixels.setPixelColor(1, 255, 0, 0); // red for invalid simp
        pixels.show();
      }
      break;
    case 5:
      Serial.println(F("CAL command received."));
      strncpy(lastCommand, "CAL", sizeof(lastCommand));
      lastCommand[sizeof(lastCommand) - 1] = '\0';
      if (ens220Initialized) {
        referencePressure = ens220.getPressureHectoPascal();
        Serial.print(F("Calibration complete. Reference pressure set to: "));
        Serial.println(referencePressure);
      } else {
        referencePressure = 1013.25;
        Serial.println(F("ENS220 not initialized, using default reference pressure: 1013.25 hPa"));
      }
      calibrateGyroscope();
      calibrateAccelerometer();
      flightState = LAUNCH_PAD;
      for (int i = 0; i < historySize; i++) {
        altitudeHistory[i] = 0.0;
        velocityHistory[i] = 0.0;
        timestampHistory[i] = 0;
      }
      maxAltitude = 0.0;
      apogeeAltitude = 0.0;
      latestVelocity = 0.0;
      Serial.println(F("History arrays and altitude variables reset."));
      pixels.setPixelColor(1, 255, 128, 0); // Orange for CAL
      pixels.show();
      break;
    case 6:
      if (strcmp(field2, "RELEASE") == 0) {
        if (strcmp(field3, "ON") == 0) {
          Serial.println(F("MEC RELEASE ON command received."));
          releaseServo.writeMicroseconds(1300);
          strncpy(lastCommand, "MEC_RELEASE_ON", sizeof(lastCommand));
          lastCommand[sizeof(lastCommand) - 1] = '\0';
          pixels.setPixelColor(4, 255, 0, 255); // Pink for MEC_RELEASE_ON
          pixels.show();
        } else if (strcmp(field3, "OFF") == 0) {
          Serial.println(F("MEC RELEASE OFF command received."));
          releaseServo.writeMicroseconds(1700);
          strncpy(lastCommand, "MEC_RELEASE_OFF", sizeof(lastCommand));
          lastCommand[sizeof(lastCommand) - 1] = '\0';
          Serial.println(F("Release mechanism deactivated - CanSat Locked in Container"));
          pixels.setPixelColor(4, 80, 0, 80); // Dark Pink for MEC_RELEASE_OFF
          pixels.show();
        }
      } else if (strcmp(field2, "CAMERA") == 0) {
        if (strcmp(field3, "BLADE") == 0) {
          if (strcmp(field4, "ON") == 0) {
            digitalWrite(BLADE_CAM_CTRL, LOW);
            Serial.println(F("MEC CAMERA BLADE ON - Camera powered ON."));
            strncpy(lastCommand, "BLADE_CAM_ON", sizeof(lastCommand));
            lastCommand[sizeof(lastCommand) - 1] = '\0';
            pixels.setPixelColor(4, 0, 255, 0); //  Green for Blade CAM ON
            pixels.show();
          } else if (strcmp(field4, "OFF") == 0) {
            digitalWrite(BLADE_CAM_CTRL, HIGH);
            Serial.println(F("MEC CAMERA BLADE OFF - Camera powered OFF."));
            strncpy(lastCommand, "BLADE_CAM_OFF", sizeof(lastCommand));
            lastCommand[sizeof(lastCommand) - 1] = '\0';
            pixels.setPixelColor(4, 0, 100, 50); // Dark Green for Blade Cam OFF
            pixels.show();
          }
        } else if (strcmp(field3, "GROUND") == 0) {
          if (strcmp(field4, "ON") == 0) {
            digitalWrite(GND_CAM_CTRL, LOW);
            Serial.println(F("MEC CAMERA GROUND ON - Camera powered ON."));
            strncpy(lastCommand, "GROUND_CAM_ON", sizeof(lastCommand));
            lastCommand[sizeof(lastCommand) - 1] = '\0';
            pixels.setPixelColor(4, 100, 255, 0); // Yellow for Ground Camera On
            pixels.show();
          } else if (strcmp(field4, "OFF") == 0) {
            digitalWrite(GND_CAM_CTRL, HIGH);
            Serial.println(F("MEC CAMERA GROUND OFF - Camera powered OFF."));
            strncpy(lastCommand, "GROUND_CAM_OFF", sizeof(lastCommand));
            lastCommand[sizeof(lastCommand) - 1] = '\0';
            pixels.setPixelColor(4, 50, 100, 0); // DARK yellow for Ground Camera Off
            pixels.show();
          }
        }
      }
      break;
    case 7:
      if (strcmp(field2, "ON") == 0) {
        Serial.println("PARTY ON command received.");
        partyMode = true;
        partyPhase = 0;
        lastPartyUpdateTime = millis();
        strncpy(lastCommand, "PARTY_ON", sizeof(lastCommand));
        lastCommand[sizeof(lastCommand) - 1] = '\0';
      } else if (strcmp(field2, "OFF") == 0) {
        Serial.println("PARTY OFF command received.");
        partyMode = false;
        strncpy(lastCommand, "PARTY_OFF", sizeof(lastCommand));
        lastCommand[sizeof(lastCommand) - 1] = '\0';
        pixels.setPixelColor(0, 0, 0, 0);
        for (int i = 1; i < NUM_LEDS; i++) {
          pixels.setPixelColor(i, 0, 0, 0);
        }
        pixels.show();
      }
      break;
    default:
      Serial.println(F("Unknown command type or transmission error"));
      pixels.setPixelColor(0, 255, 0, 0);
      pixels.show();
      break;
  }
}

void SingleShotMeasure_setup() {
  i2c_1.begin(Wire, ENS220_I2C_ADDRESS);
  ens220InitStartTime = millis();
}

void SingleShotMeasure_loop() {
  if (!ens220Initialized && millis() - ens220InitStartTime < ENS220_INIT_TIMEOUT) {
    if (millis() - ens220InitStartTime >= ENS220_INIT_INTERVAL) {
      if (ens220.begin(&i2c_1)) {
        ens220Initialized = true;
        Serial.print(F("Device UID: "));
        Serial.println(ens220.getUID(), HEX);
        ens220.setDefaultConfiguration();
        ens220.setPressureConversionTime(ENS220::PressureConversionTime::T_16_4);
        ens220.setOversamplingOfPressure(ENS220::Oversampling::N_32);
        ens220.setOversamplingOfTemperature(ENS220::Oversampling::N_32);
        ens220.setPressureTemperatureRatio(ENS220::PressureTemperatureRatio::PT_1);
        ens220.setStandbyTime(ENS220::StandbyTime::OneShotOperation);
        ens220.setPressureDataPath(ENS220::PressureDataPath::Direct);
        ens220.writeConfiguration();
      } else {
        Serial.println(F("Waiting for I2C to start"));
        pixels.setPixelColor(0, 255, 0, 0);
        pixels.show();
        ens220InitStartTime = millis();
      }
    }
  } else if (!ens220Initialized && millis() - ens220InitStartTime >= ENS220_INIT_TIMEOUT) {
    Serial.println(F("ENS220 initialization timed out"));
    ens220Initialized = true;
    pixels.setPixelColor(0, 255, 0, 0);
    pixels.show();
  }
  if (ens220Initialized) {
    ens220.singleShotMeasure(ENS220::Sensor::TemperatureAndPressure);
    ens220.waitSingleShot();
    auto result = ens220.update();
    if (result == ENS220::Result::Ok) {
      if (hasFlag(ens220.getDataStatus(), ENS220::DataStatus::PressureReady) && hasFlag(ens220.getDataStatus(), ENS220::DataStatus::TemperatureReady)) {
        pressure = ens220.getPressureHectoPascal();
        temperature = ens220.getTempCelsius();
        if (millis() % 1000 == 0) {
          Serial.print(F("P[hPa]:"));
          Serial.print(pressure);
          Serial.print(F("\tT[C]:"));
          Serial.println(temperature);
        }
        pixels.setPixelColor(0, 0, 50, 200); // blue for GPS
        pixels.show();
      }
    } else {
      pixels.setPixelColor(0, 255, 0, 0);
      pixels.show();
    }
  }
}

void updateTime(char *currentTime, size_t size) {
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

void updateFlightState(float altitude, float velocity, float x, float y, float z) {
  static unsigned long apogeeCandidateTime = 0;
  static bool apogeeCandidate = false;
  const float velocityThreshold = 1.0;
  const float accelThreshold = 2.0;
  const unsigned long apogeeConfirmWindow = 500;
  static unsigned long simStartTime = 0;
  static int simDataPoints = 0;
  static unsigned long probeReleaseTime = 0;

  // In simulation mode, require multiple data points before allowing state transitions
  if (simulationMode) {
    if (flightState == LAUNCH_PAD) {
      simDataPoints++;
      if (simDataPoints < historySize) {
        Serial.println(F("Simulation mode: Collecting initial data points, staying in LAUNCH_PAD"));
        pixels.setPixelColor(2, 255, 255, 255); // White for Launch_Pad
        pixels.show();
        lastOrientationTime = millis();
        lastOrientationX = x;
        lastOrientationY = y;
        lastOrientationZ = z;
        return;
      }
    }
  }

  // Only update lastOrientationTime if orientation changes significantly
  float orientationChange = sqrt(pow(x - lastOrientationX, 2) + pow(y - lastOrientationY, 2) + pow(z - lastOrientationZ, 2));
  if (orientationChange > 0.1) {  // Adjust threshold as needed
    lastOrientationTime = millis();
    lastOrientationX = x;
    lastOrientationY = y;
    lastOrientationZ = z;
  }

  switch (flightState) {
    case LAUNCH_PAD:
      pixels.setPixelColor(2, 255, 255, 255); // White for Launch Pad
      pixels.show();
      if (altitude > 5 && velocity > 8) {
        flightState = ASCENT;
        Serial.println(F("Flight state: ASCENT"));
        Serial.print(F("Altitude: "));
        Serial.print(altitude);
        Serial.print(F(" m, Velocity: "));
        Serial.print(velocity);
        Serial.println(F(" m/s"));
        pixels.setPixelColor(2, 0, 0, 255); //  Blue for Ascent
        pixels.show();
      }
      break;
    case ASCENT:
      pixels.setPixelColor(2, 0, 0, 255); //  Blue for Ascent
      pixels.show();
      if (altitude > maxAltitude) {
        maxAltitude = altitude;
      }
      if (altitude > 30 && velocity < -5) {
        flightState = APOGEE;
        apogeeAltitude = altitude;
        if (altitude < maxAltitude) {
          apogeeAltitude = maxAltitude;
        }
        Serial.println(F("Flight state: APOGEE"));
        Serial.print(F("Altitude: "));
        Serial.print(altitude);
        Serial.print(F(" m, Velocity: "));
        Serial.print(velocity);
        Serial.println(F(" m/s"));
        pixels.setPixelColor(2, 255, 255, 0); // Yellow for Apogee
        pixels.show();
      }
      break;
    case APOGEE:
      pixels.setPixelColor(2, 255, 255, 0); // Yellow for Apogee
      pixels.show();
      if (velocity < 0) {
        flightState = DESCENT;
        Serial.println(F("Flight state: DESCENT"));
        Serial.print(F("Altitude: "));
        Serial.print(altitude);
        Serial.print(F(" m, Velocity: "));
        Serial.print(velocity);
        Serial.println(F(" m/s"));
        pixels.setPixelColor(2, 255, 0, 0); // Red for Descent
        pixels.show();
      }
      break;
    case DESCENT:
      pixels.setPixelColor(2, 255, 0, 0); // Red for Descent
      pixels.show();
      if (!releaseActivated && altitude <= (apogeeAltitude * 0.75)) {
        flightState = PROBE_RELEASE;
        probeReleaseTime = millis();
        Serial.println(F("Flight state: PROBE_RELEASE"));
        pixels.setPixelColor(2, 255, 0, 100); // Hot Pink for Probe Release
        pixels.show();
      }
      break;
    case PROBE_RELEASE:
      pixels.setPixelColor(2, 255, 0, 100); //  Hot Pink for Probe Release
      pixels.show();
      Serial.print(F("PROBE_RELEASE: Velocity: "));
      Serial.print(velocity);
      Serial.print(F(" m/s, Time since last orientation change: "));
      Serial.println(millis() - lastOrientationTime);
      if ((abs(velocity) < 0.5)) {  // Relaxed velocity threshold and use probeReleaseTime
        landedVelocities++;
      }
      if (landedVelocities >= 10) {
        flightState = LANDED;
        landedTime = millis();
        Serial.println(F("Flight state: LANDED"));
        pixels.setPixelColor(2, 0, 255, 0); // Green for Landed
        pixels.show();
      }
      break;
    case LANDED:
      pixels.setPixelColor(2, 0, 255, 0); // Green for Landed
      pixels.show();
      break;
  }
}

void updateGPSdata(UBX_NAV_PVT_data_t *ubxDataStruct) {
  if (ubxDataStruct->numSV >= 4 && ubxDataStruct->fixType >= 3) {
    latitude = ubxDataStruct->lat / 10000000.0;
    longitude = ubxDataStruct->lon / 10000000.0;
    gpsAltitude = ubxDataStruct->hMSL / 1000.0;
    satellites = ubxDataStruct->numSV;
    snprintf(gpsTime, sizeof(gpsTime), "%02d:%02d:%02d", ubxDataStruct->hour, ubxDataStruct->min, ubxDataStruct->sec);
  } else {
    gpsAltitude = 0.0;
    satellites = 0;
    snprintf(gpsTime, sizeof(gpsTime), "00:00:00");
    pixels.setPixelColor(0, 100, 100, 0); // gold for update GPS data
    pixels.show();
  }
}

void handleSdFailureBlink() {
  if (sdFailed && millis() - lastSdBlinkTime >= SD_BLINK_INTERVAL) {
    sdLedState = !sdLedState;
    pixels.setPixelColor(0, sdLedState ? 255 : 0, 0, 0);
    pixels.show();
    lastSdBlinkTime = millis();
  }
}

void setup() {
  debugCheckpoint("CHECK? Hello World");
  pixels.begin();
  pixels.setPixelColor(0, 0, 100, 0); // green for startup
  pixels.show();
  Serial.begin(USB_BAUDRATE);
  Serial.println("hello");
  Serial1.begin(XBEE_BAUDRATE);
  Wire.begin();
  Wire.setClock(400000);
  debugCheckpoint("Coms established");
  releaseServo.attach(RELEASE_PIN);
  debugCheckpoint("Release pin good");
  Serial.println("hello2");
  if (!rtc.begin()) {
    Serial.println(F("Couldn't find DS1307 RTC!"));
    pixels.setPixelColor(0, 255, 0, 0); // red for failure of RTC
    pixels.show();
  }
  if (!rtc.isrunning()) {
    Serial.println(F("RTC is NOT running, setting time to compile time."));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    pixels.setPixelColor(0, 255, 50, 0); // red orange for RTC
    pixels.show();
  }
  Serial.println("hello3");
  pinMode(GND_CAM_CTRL, OUTPUT);
  digitalWrite(GND_CAM_CTRL, LOW);
  pinMode(BLADE_CAM_CTRL, OUTPUT);
  digitalWrite(BLADE_CAM_CTRL, LOW);
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD card initialization failed!"));
    sdFailed = true;
    pixels.setPixelColor(0, 255, 0, 0); // red for failure of Sd card
    pixels.show();
  } else {
    dataFile = SD.open("data.txt", FILE_WRITE);
    if (!dataFile) {
      Serial.println(F("Failed to open data.txt!"));
      pixels.setPixelColor(0, 255, 50, 50); // dim red for sd open
      pixels.show();
    }
    backupFile = SD.open("backup.txt", FILE_WRITE);
    if (!backupFile) {
      Serial.println(F("Failed to open backup.txt!"));
      pixels.setPixelColor(0, 255, 50, 50); // dim red for failed backup file
      pixels.show();
    }
  }
  pinMode(RPM_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmISR, RISING);

  dataFile.println("TEAM_ID,MISSION_TIME,PACKET_COUNT,MODE,STATE,ALTITUDE,TEMPERATURE,PRESSURE,VOLTAGE,GYRO_R,GYRO_P,GYRO_Y,ACCEL_R,ACCEL_P,ACCEL_Y,MAG_R,MAG_P,MAG_Y,AUTO_GYRO_ROTATION_RATE,GPS_TIME,GPS_ALTITUDE,GPS_LATITUDE,GPS_LONGITUDE,GPS_SATS,CMD_ECHO,TEAM_NAME");
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmISR, RISING);
  if (!gps.begin()) {
    Serial.println("GNSS v3 initialization failed!");
    pixels.setPixelColor(0, 255, 0, 0); // red for pin interrupt
    pixels.show();
  } else {
    gps.setI2COutput(COM_TYPE_UBX);
    gps.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
    gps.setNavigationFrequency(1);
    gps.setAutoPVTcallbackPtr(&updateGPSdata);
    pixels.setPixelColor(0, 0, 200, 200); // cyan for  I2c output
    pixels.show();
  }
  if (!lis3mdl_FC.begin_I2C(MAG_I2C_ADDRESS)) {
    Serial.println(F("Failed to find LIS3MDL #1"));
    pixels.setPixelColor(0, 255, 0, 0); // red for MAG 1 failure
    pixels.show();
  } else {
    Serial.println(F("LIS3MDL #1 Found!"));
    lis3mdl_FC.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl_FC.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl_FC.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl_FC.setRange(LIS3MDL_RANGE_4_GAUSS);
    pixels.setPixelColor(0, 0, 200, 0); // Green for MAG 1 found
    pixels.show();
  }
  if (!IMU.begin()) {
    Serial.println(F("Error initializing LSM6DS3 #1!"));
    pixels.setPixelColor(0, 255, 0, 0); // red for temp 1 failure
    pixels.show();
  } else {
    Serial.print(F("Accelerometer #1 sample rate = "));
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(F(" Hz"));
    pixels.setPixelColor(0, 0, 200, 0); // green for temp 2 found
    pixels.show();
  }
  Serial.println("hello?");
  SingleShotMeasure_setup();
}

void loop() {
  debugCheckpoint("CHECK? Hello World");

  handleSdFailureBlink();
  partyModeEffect();

  updateTime(currentTime, sizeof(currentTime));
  char command[64] = { 0 };
  static String commandBuffer = "";
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      commandBuffer.toCharArray(command, sizeof(command));
      handleCommand(command);
      commandBuffer = "";
      memset(command, 0, sizeof(command));
    } else {
      commandBuffer += c;
    }
  }

  if (calibratingGyro) calibrateGyroscope();
  if (calibratingAccel) calibrateAccelerometer();

static unsigned long lastRpmUpdate = 0;
  const unsigned long rpmUpdateInterval = 1000; // 1 second
  float rpm = 0.0;
  if (millis() - lastRpmUpdate >= rpmUpdateInterval) {
    if (rpmCount > 0 && timeDifference > 10) { // Only calculate if valid data
  rpm = (rpmCount * 60000.0) / (rpmUpdateInterval * PULSES_PER_REVOLUTION);    }
    rpmCount = 0;
    lastRpmUpdate = millis();

    // Debug output
    Serial.print("Interrupt Count: ");
    Serial.print(interruptCount);
    Serial.print(", timeDifference: ");
    Serial.print(timeDifference);
    Serial.print(" ms, RPM: ");
    Serial.println(rpm);
    Serial.print("RPM Pin State: ");
    Serial.println(digitalRead(RPM_PIN));
  }

  // Debug output
  if (millis() % 1000 == 0) { // Print every second
    Serial.print("timeDifference: ");
    Serial.print(timeDifference);
    Serial.print(" ms, rpmCount: ");
    Serial.print(rpmCount);
    Serial.print(", RPM: ");
    Serial.println(rpm);
  }
  analogReadResolution(12);
  int adcReading = analogRead(BATTERY_PIN);
  float currentVoltage = (adcReading * 8.058608e-4 / 2.647058e-1);

  gps.checkUblox();
  gps.checkCallbacks();

  sensors_event_t magEvent1;
  lis3mdl_FC.getEvent(&magEvent1);
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    pixels.setPixelColor(0, 50, 50, 200); // dim blue for accel data
    pixels.show();
  } else {
    accelX = accelY = accelZ = 0.0;
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    pixels.setPixelColor(0, 50, 200, 50); //lime green for read gyro
    pixels.show();
  } else {
    gyroX = gyroY = gyroZ = 0.0;
  }
  SingleShotMeasure_loop();

  // Calculate altitude in flight/simulation mode
  float altitude = calculateAltitude(pressure);
  if (simulationMode) {
    altitude = calculateAltitude(simulatedPressure);  // Use the same function for consistency
    Serial.print(F("Simulated Pressure: "));
    Serial.print(simulatedPressure);
    Serial.print(F(" hPa, Simulated Altitude: "));
    Serial.print(altitude);
    Serial.println(F(" m"));
  }

  if (flightState == ASCENT && altitude > maxAltitude) {
    maxAltitude = altitude;
    pixels.setPixelColor(0, 255, 200, 0); // orange for who knows whawt
    pixels.show();
  }

  if (isnan(altitude) || isinf(altitude)) altitude = 0.0;
  if (isnan(temperature) || isinf(temperature)) temperature = 0.0;
  if (isnan(pressure) || isinf(pressure)) pressure = 0.0;
  if (isnan(currentVoltage) || isinf(currentVoltage)) currentVoltage = 0.0;
  if (isnan(gyroX) || isinf(gyroX)) gyroX = 0.0;
  if (isnan(gyroY) || isinf(gyroY)) gyroY = 0.0;
  if (isnan(gyroZ) || isinf(gyroZ)) gyroZ = 0.0;
  if (isnan(accelX) || isinf(accelX)) accelX = 0.0;
  if (isnan(accelY) || isinf(accelY)) accelY = 0.0;
  if (isnan(accelZ) || isinf(accelZ)) accelZ = 0.0;
  if (isnan(magEvent1.magnetic.x) || isinf(magEvent1.magnetic.x)) magEvent1.magnetic.x = 0.0;
  if (isnan(magEvent1.magnetic.y) || isinf(magEvent1.magnetic.y)) magEvent1.magnetic.y = 0.0;
  if (isnan(magEvent1.magnetic.z) || isinf(magEvent1.magnetic.z)) magEvent1.magnetic.z = 0.0;
  if (isnan(rpm) || isinf(rpm)) rpm = 0.0;
  if (isnan(gpsAltitude) || isinf(gpsAltitude)) gpsAltitude = 0.0;
  if (isnan(latitude) || isinf(latitude)) latitude = 0.0;
  if (isnan(longitude) || isinf(longitude)) longitude = 0.0;

  if (gpsTime[0] == '\0') strncpy(gpsTime, "00:00:00", sizeof(gpsTime));
  if (lastCommand[0] == '\0') strncpy(lastCommand, "NONE", sizeof(lastCommand));
  lastCommand[sizeof(lastCommand) - 1] = '\0';  // Ensure null-termination

  // Explicitly prepare lastCommand as a string
  char lastCommandStr[16];
  strncpy(lastCommandStr, lastCommand, sizeof(lastCommandStr));
  lastCommandStr[sizeof(lastCommandStr) - 1] = '\0';  // Ensure null-termination

  if (lastTransmissionTime + 986 < millis()) {
    lastTransmissionTime = millis();
    if (telemetryEnabled) {
      char telemetry[1024];  // Increased from 512 to 1024
      const char *mode = simulationMode ? "S" : "F";
      const char *state;
      switch (flightState) {
        case LAUNCH_PAD: state = "LAUNCH_PAD"; break;
        case ASCENT: state = "ASCENT"; break;
        case APOGEE: state = "APOGEE"; break;
        case DESCENT: state = "DESCENT"; break;
        case PROBE_RELEASE: state = "PROBE_RELEASE"; break;
        case LANDED: state = "LANDED"; break;
        default: state = "UNKNOWN"; break;
      }

      pixels.setPixelColor(3, 255, 0, 0);
      pixels.show();
      snprintf(telemetry, sizeof(telemetry),
               "%s,%s,%u,%s,%s,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.0f,%s,%.1f,%.5f,%.5f,%u,%s,COSMOS",
               TEAM_ID, currentTime, packetCount, mode, state,
               altitude, temperature, pressure, currentVoltage,
               gyroX, gyroY, gyroZ, accelX, accelY, accelZ,
               magEvent1.magnetic.x, magEvent1.magnetic.y, magEvent1.magnetic.z, rpm, gpsTime, gpsAltitude, latitude, longitude, satellites, lastCommandStr);
      Serial1.println(telemetry);
      Serial.println(telemetry);
      if (dataFile && !sdFailed) {
        pixels.setPixelColor(3, 100, 0, 0); // Red for SD failed
        pixels.show();
        dataFile.println(telemetry);
        dataFile.flush();
      }
      if (backupFile && !sdFailed) {
        backupFile.println(telemetry);
        backupFile.flush();
      }
      packetCount++;
      static unsigned long lastFileClose = 0;
      if (millis() - lastFileClose > 60000) {
        if (dataFile && !sdFailed) {
          dataFile.close();
          dataFile = SD.open("data.txt", FILE_WRITE);
        }
        if (backupFile && !sdFailed) {
          backupFile.close();
          backupFile = SD.open("backup.txt", FILE_WRITE);
        }
        lastFileClose = millis();
        pixels.setPixelColor(0, 50, 50, 50); // gray for file close
        pixels.show();
      }
    }
  }

  updateAltitudeHistory(altitudeHistory, timestampHistory, altitude, historySize);
  updateVelocityHistory(altitudeHistory, velocityHistory, timestampHistory, historySize);
  switch (flightState) {
    case LAUNCH_PAD:
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      break;
    case ASCENT:
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      break;
    case APOGEE:
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      break;
    case DESCENT:
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      break;
    case PROBE_RELEASE:
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      releaseServo.writeMicroseconds(1300);
      releaseActivated = true;
      pixels.setPixelColor(2, 255, 0, 100); // Hot pink for Probe Release
      pixels.show();
      break;
    case LANDED:
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      if (avg(velocityHistory, historySize) < 1) {
        flightState = LANDED;
      }
      break;
  }
}
