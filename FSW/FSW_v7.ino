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

// PINS AND DEFINITIONS
#define BATTERY_PIN A2          // Analog pin for voltage divider circuit
#define RPM_PIN A4              // Pin for Hall effect sensor
#define SD_CS_PIN 6             // Chip select pin for SD card
#define RELEASE_PIN 13          // Release servo pin
#define ENS220_I2C_ADDRESS 0x20 // I2C Address for ENS 220
#define DS1307_I2C_ADDRESS 0x68
#define MAG_I2C_ADDRESS 0x1E    // I2C Address for DS1307
#define USB_BAUDRATE 115200 
#define XBEE_BAUDRATE 115200    // Reduced baud rate for stability
#define GND_CAM_CTRL 11         // Blade camera
#define BLADE_CAM_CTRL 10       // Ground camera
#define LED_DATA 5
#define NUM_LEDS 5              // Number of LEDs for NeoPixel
#define TEAM_ID "3195"

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
FlightState flightState = LAUNCH_PAD;  // Initial state
Adafruit_NeoPixel pixels(NUM_LEDS, LED_DATA, NEO_GRB + NEO_KHZ800);
ScioSense::ENS220 ens220;
I2cInterface i2c_1;            // For ENS220 single-shot mode
Adafruit_LIS3MDL lis3mdl_FC;   // First magnetometer
Servo releaseServo;
File dataFile;                 // Global SD file handle
File backupFile;               // Global backup file handle
SFE_UBLOX_GNSS gps;
RTC_DS1307 rtc;

// Variables
unsigned long landedTime = 0;
unsigned long lastOrientationTime = 0;
float lastOrientationX = 0.0, lastOrientationY = 0.0, lastOrientationZ = 0.0;
uint8_t satellites = 0;
float lastTransmissionTime = 0;                          // Last telemetry transmission
char currentTime[9] = "00:00:00";                        // Mission time in "HH:MM:SS"
char gpsTime[9] = "00:00:00";                            // GPS time in "HH:MM:SS"
float accel2X, accel2Y, accel2Z, gyro2X, gyro2Y, gyro2Z; // IMU data
char lastCommand[32] = "NONE";                           // Last received command, initialized
unsigned int packetCount = 1;
bool telemetryEnabled = false;  // Telemetry control
float cameraposition = 0;
unsigned long lastRpmTime = 0;        // Last magnet detection
volatile unsigned long rpmCount = 0;  // RPM counter
volatile float currentInterruptTime = 0;  // Current interrupt time
volatile float timeDifference = 0;         // Time between interrupts
volatile float lastInterruptTime = 0;
float apogeeAltitude = 0.0;
float maxAltitude = 0;
bool releaseActivated = false;
bool simulationMode = false;
float simulatedPressure = 0.0;
float receivedPressure = 0.0;       // For SIM_ACTIVATE
float referencePressure = 1013.25;  // Sea level pressure
const int historySize = 10;
float pressure;
float temperature;
float altitudeHistory[historySize];           // Store altitude
float velocityHistory[historySize];           // Store velocity
unsigned long timestampHistory[historySize];  // Store time
float latestVelocity;

// Calibration biases for gyroscope and accelerometer
float gyroBiasX = 0.0, gyroBiasY = 0.0, gyroBiasZ = 0.0;
float accelBiasX = 0.0, accelBiasY = 0.0, accelBiasZ = 0.0;

// Variables to store GPS data
double latitude = 0.0;
double longitude = 0.0;
double gpsAltitude = 0.0;

// Non-blocking ENS220 initialization variables
bool ens220Initialized = false;
unsigned long ens220InitStartTime = 0;
const unsigned long ENS220_INIT_TIMEOUT = 10000; // 10 seconds timeout
const unsigned long ENS220_INIT_INTERVAL = 1000; // Try every 1 second

// Non-blocking SD failure handling
bool sdFailed = false;
unsigned long lastSdBlinkTime = 0;
bool sdLedState = false;
const unsigned long SD_BLINK_INTERVAL = 500; // Blink every 500 ms

// Non-blocking calibration variables
bool calibratingGyro = false;
bool calibratingAccel = false;
unsigned long lastCalibrationSampleTime = 0;
const unsigned long CALIBRATION_SAMPLE_INTERVAL = 10; // 10 ms between samples
const int CALIBRATION_SAMPLES = 100;
int calibrationSampleCount = 0;
float calibSumX = 0.0, calibSumY = 0.0, calibSumZ = 0.0;

// Function Definitions
float calculateAltitude(float pressure) {
  const float temperatureLapseRate = 0.0065;
  const float seaLevelTemperature = 288.15;
  const float gasConstant = 8.3144598;
  const float molarMass = 0.0289644;
  const float gravity = 9.80665;
  float altitude = (seaLevelTemperature / temperatureLapseRate) * 
                   (1 - pow((pressure / referencePressure), 
                            (gasConstant * temperatureLapseRate) / (gravity * molarMass)));
  pixels.setPixelColor(0, 0, 255, 255); // Cyan for sea level pressure
  pixels.show();
  return altitude;
}

void calibrateGyroscope() {
  if (!calibratingGyro) {
    calibratingGyro = true;
    calibrationSampleCount = 0;
    calibSumX = 0.0;
    calibSumY = 0.0;
    calibSumZ = 0.0;
    lastCalibrationSampleTime = millis();
    Serial.println("Calibrating gyroscope...");
    pixels.setPixelColor(0, 255, 128, 0); // Orange for Gyro CAL
    pixels.show();
  }

  if (millis() - lastCalibrationSampleTime >= CALIBRATION_SAMPLE_INTERVAL) {
    if (IMU.gyroscopeAvailable() && calibrationSampleCount < CALIBRATION_SAMPLES) {
      float gx, gy, gz;
      IMU.readGyroscope(gx, gy, gz);
      calibSumX += gx;
      calibSumY += gy;
      calibSumZ += gz;
      calibrationSampleCount++;
      lastCalibrationSampleTime = millis();
    }

    if (calibrationSampleCount >= CALIBRATION_SAMPLES) {
      gyroBiasX = calibSumX / CALIBRATION_SAMPLES;
      gyroBiasY = calibSumY / CALIBRATION_SAMPLES;
      gyroBiasZ = calibSumZ / CALIBRATION_SAMPLES;

      Serial.print("Gyroscope bias: ");
      Serial.print(gyroBiasX);
      Serial.print(", ");
      Serial.print(gyroBiasY);
      Serial.print(", ");
      Serial.println(gyroBiasZ);
      pixels.setPixelColor(0, 255, 128, 0); // Orange for completion of Gyro CAL
      pixels.show();
      calibratingGyro = false;
    }
  }
}

void calibrateAccelerometer() {
  if (!calibratingAccel) {
    calibratingAccel = true;
    calibrationSampleCount = 0;
    calibSumX = 0.0;
    calibSumY = 0.0;
    calibSumZ = 0.0;
    lastCalibrationSampleTime = millis();
    Serial.println("Calibrating accelerometer...");
    pixels.setPixelColor(0, 255, 69, 0); // Red Orange for Accel CAL
    pixels.show();
  }

  if (millis() - lastCalibrationSampleTime >= CALIBRATION_SAMPLE_INTERVAL) {
    if (IMU.accelerationAvailable() && calibrationSampleCount < CALIBRATION_SAMPLES) {
      float ax, ay, az;
      IMU.readAcceleration(ax, ay, az);
      calibSumX += ax;
      calibSumY += ay;
      calibSumZ += az;
      calibrationSampleCount++;
      lastCalibrationSampleTime = millis();
    }

    if (calibrationSampleCount >= CALIBRATION_SAMPLES) {
      accelBiasX = calibSumX / CALIBRATION_SAMPLES;
      accelBiasY = calibSumY / CALIBRATION_SAMPLES;
      accelBiasZ = (calibSumZ / CALIBRATION_SAMPLES) - 9.81; // Subtract gravity (1g)

      Serial.print("Accelerometer bias: ");
      Serial.print(accelBiasX);
      Serial.print(", ");
      Serial.print(accelBiasY);
      Serial.print(", ");
      Serial.println(accelBiasZ);
      pixels.setPixelColor(0, 255, 69, 0); // Red Orange for completion for Accel CAL
      pixels.show();
      calibratingAccel = false;
    }
  }
}

void updateAltitudeHistory(float altitudeHistory[], unsigned long timestampHistory[], float newAltitude, int size) {
  for (int i = size - 1; i > 0; i--) {
    altitudeHistory[i] = altitudeHistory[i - 1];
    timestampHistory[i] = timestampHistory[i - 1];
  }
  altitudeHistory[0] = newAltitude;
  timestampHistory[0] = millis();
  pixels.setPixelColor(0, 90, 225, 100); // Teal for Updating Altitude History
  pixels.show();
}

void updateVelocityHistory(float altitudeHistory[], float velocityHistory[], unsigned long timestampHistory[], int size) {
  unsigned long timeDifferenceMillis = timestampHistory[0] - timestampHistory[1];
  float timeDifferenceSeconds = timeDifferenceMillis / 1000.0;
  latestVelocity = (altitudeHistory[0] - altitudeHistory[1]) / timeDifferenceSeconds;
  for (int i = size - 1; i > 0; i--) {
    velocityHistory[i] = velocityHistory[i - 1];
  }
  velocityHistory[0] = latestVelocity;
  pixels.setPixelColor(0, 240, 25, 50); // Light red for Updating velocity History
  pixels.show();
}

float avg(float arr[], int size) {
  float sum = 0.0;
  for (int i = 0; i < size; i++) {
    sum += arr[i];
  }
  return sum / size;
}

void rpmISR() {
  static unsigned long lastInterrupt = 0;
  unsigned long now = micros();
  if (now - lastInterrupt > 1000) {
    currentInterruptTime = now;
    timeDifference = (currentInterruptTime - lastInterruptTime) / 1000.0; // ms
    lastInterruptTime = currentInterruptTime;
    rpmCount++;
    lastInterrupt = now;
    pixels.setPixelColor(0, 255, 255, 3); // Yellow for RPM
    pixels.show();
  }
}

void handleCommand(const char *command) {
  char field1[10], field2[10], field3[10], field4[10];
  int num = sscanf(command, "CMD, %*d, %9[^,], %9[^,], %9[^,], %9[^,]",
                   field1, field2, field3, field4);
  if (num < 1) {
    Serial.println(F("Invalid command"));
    pixels.setPixelColor(0, 255, 0, 0); // Red for Invalid command
    pixels.show();
    return;
  }
  float tempPressure;
  int cmdType = 0;
  if (strcmp(field1, "CX") == 0) cmdType = 1;
  else if (strcmp(field1, "ST") == 0) cmdType = 2;
  else if (strcmp(field1, "SIM") == 0) cmdType = 3;
  else if (strcmp(field1, "SIMP") == 0) cmdType = 4;
  else if (strcmp(field1, "CAL") == 0) cmdType = 5;
  else if (strcmp(field1, "MEC") == 0) cmdType = 6;

  switch (cmdType) {
    case 1:
      if (strcmp(field2, "ON") == 0) {
        telemetryEnabled = true;
        strncpy(lastCommand, "CXON", sizeof(lastCommand));
        Serial.println(F("Telemetry started."));
        pixels.setPixelColor(0, 0, 255, 0); // Green for CX_ON
        pixels.show();
      } else if (strcmp(field2, "OFF") == 0) {
        telemetryEnabled = false;
        strncpy(lastCommand, "CXOFF", sizeof(lastCommand));
        Serial.println(F("Telemetry stopped."));
        pixels.setPixelColor(0, 128, 128, 128); // Gray for CX_OFF
        pixels.show();
      }
      break;
    case 2:
      if (strcmp(field2, "UTC_TIME") == 0) {
        Serial.println(F("ST UTC_TIME command received."));
        strncpy(currentTime, field3, sizeof(currentTime));
        strncpy(lastCommand, "ST_UTC_TIME", sizeof(lastCommand));
        pixels.setPixelColor(0, 0, 50, 255); // Blue UTC time
        pixels.show();
      } else if (strcmp(field2, "GPS") == 0) {
        Serial.println(F("ST GPS command received."));
        strncpy(currentTime, gpsTime, sizeof(currentTime));
        strncpy(lastCommand, "ST_GPS", sizeof(lastCommand));
        pixels.setPixelColor(0, 140, 50, 0); // Brown light for GPS
        pixels.show();
      }
      break;
    case 3:
      if (strcmp(field2, "ENABLE") == 0) {
        simulationMode = true;
        Serial.println(F("Simulation mode enabled."));
        strncpy(lastCommand, "SIM_ENABLE", sizeof(lastCommand));
        pixels.setPixelColor(0, 255, 0, 255); // Light Magenta light for SIM_ENABLE
        pixels.show();
      } else if (strcmp(field2, "ACTIVATE") == 0 && simulationMode) {
        Serial.println(F("Simulation activated. Waiting for pressure input..."));
        strncpy(lastCommand, "SIM_ACTIVATE", sizeof(lastCommand));
        pixels.setPixelColor(0, 255, 0, 150); // Dark Magenta light for SIM_ACTIVATE
        pixels.show();
        char pressureInput[16] = {0};
        unsigned long start = millis();
        size_t bytesRead = 0;
        while (millis() - start < 1000 && bytesRead < sizeof(pressureInput) - 1) {
          if (Serial1.available()) {
            pressureInput[bytesRead] = Serial1.read();
            if (pressureInput[bytesRead] == '\n') {
              pressureInput[bytesRead] = '\0';
              break;
            }
            bytesRead++;
          }
        }
        pressureInput[bytesRead] = '\0';
        receivedPressure = atof(pressureInput);
        if (receivedPressure > 0.0 && receivedPressure < 2000.0) {
          simulatedPressure = receivedPressure;
          Serial.print(F("Simulated pressure updated: "));
          Serial.println(simulatedPressure);
          pixels.setPixelColor(0, 85, 155, 55); // Light Green for Simulated Pressure
          pixels.show();
        } else {
          Serial.println(F("Invalid pressure value received."));
          pixels.setPixelColor(0, 150, 0, 0); // Maroon for Invalid Pressure
          pixels.show();
        }
      } else if (strcmp(field2, "DISABLE") == 0) {
        simulationMode = false;
        Serial.println(F("Simulation mode disabled."));
        strncpy(lastCommand, "SIM_DISABLE", sizeof(lastCommand));
        pixels.setPixelColor(0, 250, 140, 140); // White Red for Disable Simulation
        pixels.show();
      }
      break;
    case 4:
      tempPressure = atof(field2);
      if (tempPressure > 0.0) {
        simulatedPressure = tempPressure;
        Serial.println(F("Simulated pressure set via SIMP."));
        pixels.setPixelColor(0, 90, 20, 150); // Light purple for SIMP Pressure
        pixels.show();
        strncpy(lastCommand, "SIMP", sizeof(lastCommand));
      } else {
        Serial.println(F("Invalid pressure value in SIMP command."));
        pixels.setPixelColor(0, 40, 0, 80); // Purple for invalid SIMP Pressure
        pixels.show();
      }
      break;
    case 5:
      Serial.println(F("CAL command received."));
      strncpy(lastCommand, "CAL", sizeof(lastCommand));
      referencePressure = ens220.getPressureHectoPascal();
      Serial.print(F("Calibration complete. Reference pressure set to: "));
      Serial.println(referencePressure);
      flightState = LAUNCH_PAD;
      pixels.setPixelColor(0, 60, 80, 0); // Turquoise for CAL command
      pixels.show();
      break;
    case 6:
      if (strcmp(field2, "RELEASE") == 0) {
        if (strcmp(field3, "ON") == 0) {
          Serial.println(F("MEC RELEASE ON command received."));
          releaseServo.writeMicroseconds(1375);
          strncpy(lastCommand, "MEC_RELEASE_ON", sizeof(lastCommand));
          pixels.setPixelColor(0, 180, 120, 0); // Dark Yellow for MEC Release ON
          pixels.show();
        } else if (strcmp(field3, "OFF") == 0) {
          Serial.println(F("MEC RELEASE OFF command received."));
          releaseServo.writeMicroseconds(1600);
          strncpy(lastCommand, "MEC_RELEASE_OFF", sizeof(lastCommand));
          Serial.println(F("Release mechanism deactivated - CanSat Locked in Container"));
          pixels.setPixelColor(0, 50, 120, 255); // Light Blue for MEC Release OFF
          pixels.show();
        }
      } else if (strcmp(field2, "CAMERA") == 0) {
        if (strcmp(field3, "BLADE") == 0) {
          if (strcmp(field4, "ON") == 0) {
            digitalWrite(BLADE_CAM_CTRL, LOW);
            Serial.println(F("MEC CAMERA BLADE ON - Camera powered ON."));
            strncpy(lastCommand, "BLADE_CAM_CTRL_ON", sizeof(lastCommand));
            pixels.setPixelColor(0, 0, 255, 0); // Dark Green for Blade CAM on
            pixels.show();
          } else if (strcmp(field4, "OFF") == 0) {
            digitalWrite(BLADE_CAM_CTRL, HIGH);
            Serial.println(F("MEC CAMERA BLADE OFF - Camera powered OFF."));
            strncpy(lastCommand, "BLADE_CAM_CTRL_OFF", sizeof(lastCommand));
            pixels.setPixelColor(0, 0, 100, 50); // Leaf Green for Blade CAM off
            pixels.show();
          }
        } else if (strcmp(field3, "GROUND") == 0) {
          if (strcmp(field4, "ON") == 0) {
            digitalWrite(GND_CAM_CTRL, HIGH);
            Serial.println(F("MEC CAMERA GROUND ON - Camera powered ON."));
            strncpy(lastCommand, "GROUND_CAM_ON", sizeof(lastCommand));
            pixels.setPixelColor(0, 100, 255, 0); // Lime for Ground CAM on
            pixels.show();
          } else if (strcmp(field4, "OFF") == 0) {
            digitalWrite(GND_CAM_CTRL, LOW);
            Serial.println(F("MEC CAMERA GROUND OFF - Camera powered OFF."));
            strncpy(lastCommand, "GROUND_CAM_OFF", sizeof(lastCommand));
            pixels.setPixelColor(0, 50, 100, 0); // Dim Lime for Ground CAM off
            pixels.show();
          }
        }
      }
      break;
    default:
      Serial.println(F("Unknown command type or transmission error"));
      pixels.setPixelColor(0, 255, 0, 0); // Red for Unknown command
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
        pixels.setPixelColor(0, 0, 200, 0); // Green for ENS220 setup complete
        pixels.show();
      } else {
        Serial.println(F("Waiting for I2C to start"));
        pixels.setPixelColor(0, 255, 0, 0); // Red for I2C failure
        pixels.show();
        ens220InitStartTime = millis(); // Reset interval for next attempt
      }
    }
  } else if (!ens220Initialized && millis() - ens220InitStartTime >= ENS220_INIT_TIMEOUT) {
    Serial.println(F("ENS220 initialization timed out"));
    ens220Initialized = true; // Proceed to avoid infinite attempts
    pixels.setPixelColor(0, 255, 0, 0); // Red for timeout
    pixels.show();
  }

  if (ens220Initialized) {
    ens220.singleShotMeasure(ENS220::Sensor::TemperatureAndPressure);
    ens220.waitSingleShot();
    auto result = ens220.update();
    if (result == ENS220::Result::Ok) {
      if (hasFlag(ens220.getDataStatus(), ENS220::DataStatus::PressureReady) && 
          hasFlag(ens220.getDataStatus(), ENS220::DataStatus::TemperatureReady)) {
        pressure = ens220.getPressureHectoPascal();
        temperature = ens220.getTempCelsius();
        if (millis() % 1000 == 0) {
          Serial.print(F("P[hPa]:"));
          Serial.print(pressure);
          Serial.print(F("\tT[C]:"));
          Serial.println(temperature);
        }
        pixels.setPixelColor(0, 0, 50, 200); // Blue-green for successful measurement
        pixels.show();
      }
    } else {
      pixels.setPixelColor(0, 255, 0, 0); // Red for measurement failure
      pixels.show();
    }
  }
}

void updateTime(char *currentTime, size_t size) {
  if (rtc.isrunning()) {
    DateTime now = rtc.now();
    snprintf(currentTime, size, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    pixels.setPixelColor(0, 0, 0, 100); // Dim blue for RTC time update
    pixels.show();
  } else {
    Serial.println(F("RTC not running, using fallback time."));
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
      pixels.setPixelColor(0, 100, 0, 0); // Dim red for fallback time
      pixels.show();
    }
  }
}

void updateFlightState(float altitude, float velocity, float x, float y, float z) {
  static unsigned long apogeeCandidateTime = 0; // Time when apogee conditions first met
  static bool apogeeCandidate = false; // Flag to track apogee candidate
  const float velocityThreshold = 1.0; // Velocity range for apogee (±1 m/s)
  const float accelThreshold = 2.0; // Acceleration threshold for apogee (±2 m/s²)
  const unsigned long apogeeConfirmWindow = 500; // Confirmation window (ms)

  switch (flightState) {
    case LAUNCH_PAD:
      pixels.setPixelColor(0, 255, 255, 255); // White for LAUNCH_PAD
      pixels.show();
      if (altitude > 5 && velocity > 8) {
        flightState = ASCENT;
        Serial.println(F("Flight state: ASCENT"));
        pixels.setPixelColor(0, 0, 255, 0); // Green for ASCENT
        pixels.show();
      }
      break;
    case ASCENT: {
      pixels.setPixelColor(0, 0, 255, 0); // Green for ASCENT
      pixels.show();
      if (altitude > maxAltitude) {
        maxAltitude = altitude;
      }

      bool velocityCondition = abs(velocity) <= velocityThreshold;
      bool altitudeCondition = altitude <= maxAltitude && altitudeHistory[0] <= altitudeHistory[1];
      bool accelCondition = abs(z - accelBiasZ) <= accelThreshold;

      if (velocityCondition && altitudeCondition && accelCondition) {
        if (!apogeeCandidate) {
          apogeeCandidate = true;
          apogeeCandidateTime = millis();
        } else if (millis() - apogeeCandidateTime >= apogeeConfirmWindow) {
          flightState = APOGEE;
          apogeeAltitude = altitude;
          apogeeCandidate = false;
          Serial.println(F("Flight state: APOGEE"));
          pixels.setPixelColor(0, 255, 255, 0); // Yellow for APOGEE
          pixels.show();
        }
      } else {
        apogeeCandidate = false;
      }
      break;
    }
    case APOGEE:
      pixels.setPixelColor(0, 255, 255, 0); // Yellow for APOGEE
      pixels.show();
      if (velocity < 0) {
        flightState = DESCENT;
        Serial.println(F("Flight state: DESCENT"));
        pixels.setPixelColor(0, 255, 0, 0); // Red for DESCENT
        pixels.show();
      }
      break;
    case DESCENT:
      pixels.setPixelColor(0, 255, 0, 0); // Red for DESCENT
      pixels.show();
      if (!releaseActivated && altitude <= (apogeeAltitude * 0.75)) {
        flightState = PROBE_RELEASE;
        Serial.println(F("Flight state: PROBE_RELEASE"));
        pixels.setPixelColor(0, 255, 0, 100); // Pink for PROBE_RELEASE
        pixels.show();
      }
      break;
    case PROBE_RELEASE:
      pixels.setPixelColor(0, 255, 0, 100); // Pink for PROBE_RELEASE
      pixels.show();
      if (abs(velocity) < 0.1 && millis() - lastOrientationTime > 10000) {
        flightState = LANDED;
        landedTime = millis();
        Serial.println(F("Flight state: LANDED"));
        pixels.setPixelColor(0, 128, 0, 128); // Purple for LANDED
        pixels.show();
      }
      break;
    case LANDED:
      pixels.setPixelColor(0, 128, 0, 128); // Purple for LANDED
      pixels.show();
      break;
  }
  lastOrientationTime = millis();
  lastOrientationX = x;
  lastOrientationY = y;
  lastOrientationZ = z;
}

void updateGPSdata(UBX_NAV_PVT_data_t *ubxDataStruct) {
  if (ubxDataStruct->numSV >= 4 && ubxDataStruct->fixType >= 3) {
    latitude = ubxDataStruct->lat / 10000000.0;
    longitude = ubxDataStruct->lon / 10000000.0;
    gpsAltitude = ubxDataStruct->hMSL / 1000.0;
    satellites = ubxDataStruct->numSV;
    snprintf(gpsTime, sizeof(gpsTime), "%02d:%02d:%02d", ubxDataStruct->hour, ubxDataStruct->min, ubxDataStruct->sec);
    pixels.setPixelColor(0, 0, 200, 200); // Cyan for valid GPS fix
    pixels.show();
  } else {
    latitude = longitude = gpsAltitude = 0.0;
    satellites = 0;
    snprintf(gpsTime, sizeof(gpsTime), "00:00:00");
    pixels.setPixelColor(0, 100, 100, 0); // Dim yellow for no GPS fix
    pixels.show();
  }
}

void handleSdFailureBlink() {
  if (sdFailed && millis() - lastSdBlinkTime >= SD_BLINK_INTERVAL) {
    sdLedState = !sdLedState;
    pixels.setPixelColor(0, sdLedState ? 255 : 0, 0, 0); // Red or off
    pixels.show();
    lastSdBlinkTime = millis();
  }
}

void setup() {
  pixels.begin();
  pixels.setPixelColor(0, 0, 100, 0); // Green for startup
  pixels.show();

  Serial.begin(USB_BAUDRATE);
  Serial.println("hello");
  Serial1.begin(XBEE_BAUDRATE);
  Wire.begin();
  Wire.setClock(400000);

  releaseServo.attach(RELEASE_PIN);

  Serial.println("hello2");
  if (!rtc.begin()) {
    Serial.println(F("Couldn't find DS1307 RTC!"));
    pixels.setPixelColor(0, 255, 0, 0); // Red for RTC failure
    pixels.show();
  }
  if (!rtc.isrunning()) {
    Serial.println(F("RTC is NOT running, setting time to compile time."));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    pixels.setPixelColor(0, 255, 50, 0); // Orange for RTC time set
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
    pixels.setPixelColor(0, 255, 0, 0); // Red for SD failure
    pixels.show();
  } else {
    Serial.println(F("SD card initialized successfully"));
    dataFile = SD.open("data.txt", FILE_WRITE);
    if (!dataFile) {
      Serial.println(F("Failed to open data.txt!"));
      pixels.setPixelColor(0, 255, 50, 50); // Pink for file open failure
      pixels.show();
    }
    backupFile = SD.open("backup.txt", FILE_WRITE);
    if (!backupFile) {
      Serial.println(F("Failed to open backup.txt!"));
      pixels.setPixelColor(0, 255, 50, 50); // Pink for file open failure
      pixels.show();
    }
    pixels.setPixelColor(0, 0, 200, 0); // Green for SD success
    pixels.show();
  }

  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmISR, RISING);

  if (!gps.begin()) {
    Serial.println("GNSS v3 initialization failed!");
    pixels.setPixelColor(0, 255, 0, 0); // Red for GPS failure
    pixels.show();
  } else {
    gps.setI2COutput(COM_TYPE_UBX);
    gps.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
    gps.setNavigationFrequency(1);
    gps.setAutoPVTcallbackPtr(&updateGPSdata);
    pixels.setPixelColor(0, 0, 200, 200); // Cyan for GPS success
    pixels.show();
  }

  if (!lis3mdl_FC.begin_I2C(MAG_I2C_ADDRESS)) {
    Serial.println(F("Failed to find LIS3MDL #1"));
    pixels.setPixelColor(0, 255, 0, 0); // Red for magnetometer failure
    pixels.show();
  } else {
    Serial.println(F("LIS3MDL #1 Found!"));
    lis3mdl_FC.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl_FC.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl_FC.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl_FC.setRange(LIS3MDL_RANGE_4_GAUSS);
    pixels.setPixelColor(0, 0, 200, 0); // Green for magnetometer success
    pixels.show();
  }

  if (!IMU.begin()) {
    Serial.println(F("Error initializing LSM6DS3 #1!"));
    pixels.setPixelColor(0, 255, 0, 0); // Red for IMU failure
    pixels.show();
  } else {
    Serial.print(F("Accelerometer #1 sample rate = "));
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(F(" Hz"));
    pixels.setPixelColor(0, 0, 200, 0); // Green for IMU success
    pixels.show();
  }

  Serial.println("hello?");
  Serial.println("telemetryEnabled: " + String(telemetryEnabled));
  SingleShotMeasure_setup();
}

void loop() {
  handleSdFailureBlink();

  updateTime(currentTime, sizeof(currentTime));
  char command[64] = {0};
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

  unsigned long missionTime = millis() / 1000;
  float rpm = (timeDifference > 0) ? (60000 / timeDifference) : 0;
  lastRpmTime = millis();
  rpmCount = 0;
  int adcReading = analogRead(BATTERY_PIN);
  float currentVoltage = ((adcReading * 8.058608e-4) / 2.647058e-1);

  gps.checkUblox();
  gps.checkCallbacks();
  
  sensors_event_t magEvent1;
  lis3mdl_FC.getEvent(&magEvent1);
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    if (millis() % 1000 == 0) {
      Serial.print(accelX);
      Serial.print(F("\t"));
      Serial.print(accelY);
      Serial.print(F("\t"));
      Serial.println(accelZ);
    }
    pixels.setPixelColor(0, 50, 50, 200); // Blue for accelerometer data
    pixels.show();
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    pixels.setPixelColor(0, 50, 200, 50); // Green for gyroscope data
    pixels.show();
  }
  SingleShotMeasure_loop();

  float altitude = calculateAltitude(pressure);
  if (flightState == ASCENT && altitude > maxAltitude) {
    maxAltitude = altitude;
    pixels.setPixelColor(0, 255, 200, 0); // Bright yellow for new max altitude
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

  if (lastTransmissionTime + 986 < millis()) {
    lastTransmissionTime = millis();
    if (telemetryEnabled) {
      char telemetry[512];
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
      
      int currentVoltage_int = (int)(currentVoltage * 10);
      int gyroX_int = (int)(gyroX * 10);
      int gyroY_int = (int)(gyroY * 10);
      int gyroZ_int = (int)(gyroZ * 10);
      int accelX_int = (int)(accelX * 10);
      int accelY_int = (int)(accelY * 10);
      int accelZ_int = (int)(accelZ * 10);
      int magX_int = (int)(magEvent1.magnetic.x * 10);
      int magY_int = (int)(magEvent1.magnetic.y * 10);
      int magZ_int = (int)(magEvent1.magnetic.z * 10);
      int rpm_int = (int)(rpm * 10);

      pixels.setPixelColor(4, 255, 0, 0); // Red for telemetry transmission
      pixels.show();
      snprintf(telemetry, sizeof(telemetry),
               "%s,%s,%u,%s,%s,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%.1f,%.4f,%.4f,%u,%s,COSMOS",
               TEAM_ID, currentTime, packetCount, mode, state,
               altitude, temperature, pressure, currentVoltage_int,
               gyroX_int, gyroY_int, gyroZ_int, accelX_int, accelY_int, accelZ_int,
               magX_int, magY_int, magZ_int, rpm_int, gpsTime, gpsAltitude,
               latitude, longitude, satellites, lastCommand);
      Serial1.println(telemetry);
      Serial.println(telemetry);
      if (dataFile && !sdFailed) {
        pixels.setPixelColor(2, 100, 0, 0); // Dim red for SD write
        pixels.show();
        dataFile.println(telemetry);
        dataFile.flush();
        pixels.setPixelColor(2, 0, 0, 0);
        pixels.show();
      }
      if (backupFile && !sdFailed) {
        backupFile.println(telemetry);
        backupFile.flush();
      }
      packetCount++;
      pixels.setPixelColor(4, 0, 0, 0); // Clear telemetry LED
      pixels.show();
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
        pixels.setPixelColor(0, 50, 50, 50); // Dim white for file reopen
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
      if (!releaseActivated && altitude <= (apogeeAltitude * 0.75)) {
        releaseServo.writeMicroseconds(1375);
        releaseActivated = true;
        pixels.setPixelColor(0, 255, 0, 100); // Pink for release activation
        pixels.show();
      }
      break;
    case PROBE_RELEASE:
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      break;
    case LANDED:
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      if (avg(velocityHistory, historySize) < 1) {
        flightState = LANDED;
      }
      break;
  }
}
