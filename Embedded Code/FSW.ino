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
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Arduino.h>
#include "i2c_interface.h"
#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include <FeedBackServo.h>
#include <RTClib.h>

// PINS AND DEFINITIONS
#define BATTERY_PIN A0         // Analog pin for voltage divider circuit
#define RPM_PIN 2              // Pin for Hall effect sensor (D2 on Arduino Zero)
#define SD_CS_PIN 6            // Chip select pin for SD card (adjust for your board)
#define SERVO_PIN 13           // Servo pin for GND camera stabilization (D13 on Zero)
#define FEEDBACK_PIN 9         // Feedback signal pin (D9 on Zero, unused here)
#define I2C_ADDRESS 0x20       // I2C Address for ENS 220
#define DS1307_I2C_ADDRESS 0x68 // I2C Address for DS1307
#define SERIAL_BAUDRATE 57600  // Speed of Serial Communication with the computer (ENS220)
#define INTN_1 4               // Interrupt pin for ENS220 (D4 on Zero)
#define CAMERA1_PIN 10         // Blade camera (D10 on Zero)
#define CAMERA2_PIN 11         // Ground camera (D11 on Zero)
#define LED_DATA 5             // NeoPixel data pin (D5 on Zero)
#define BUZZER_PIN 3           // Buzzer pin (D3 on Zero, PA09)
#define NUM_LEDS 5             // Number of LEDs for NeoPixel
#define MAG1_I2C_ADDRESS 0x1C  // First LIS3MDL address
#define MAG2_I2C_ADDRESS 0x1E  // Second LIS3MDL address
#define IMU1_I2C_ADDRESS 0x6A  // First LSM6DS3 address
#define IMU2_I2C_ADDRESS 0x6B  // Second LSM6DS3 address
#define TEAM_ID "3195"         // Team ID
#define XBEE_TX 1              // XBee TX connected to D1
#define XBEE_RX 0              // XBee RX connected to D0

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
ScioSense::ENS220 ens220;
I2cInterface i2c_1;            // Added for ENS220 single-shot mode
Adafruit_LIS3MDL lis3mdl;      // Magnetometer
Adafruit_LIS3MDL lis3mdl_FC;   // First magnetometer
Adafruit_LIS3MDL lis3mdl_CAM;  // Second magnetometer
Servo servo;
File dataFile;
File backupFile;
SFE_UBLOX_GNSS gps;            // GPS object
RTC_DS1307 rtc;                // DS1307 RTC

// Variables
unsigned long landedTime = 0;
unsigned long lastOrientationTime = 0;
float lastOrientationX = 0.0, lastOrientationY = 0.0, lastOrientationZ = 0.0;
uint8_t satellites = 0;
float voltageDividerFactor = 0.012089;                    // Adjust based on resistor values
float lastTransmissionTime = 0;                           // Last telemetry transmission time
char currentTime[9] = "00:00:00";                         // Mission time in "HH:MM:SS"
char gpsTime[9] = "00:00:00";                             // GPS time in "HH:MM:SS"
float mag2X, mag2Y, mag2Z;                                // Magnetometer data for second sensor
float accel2X, accel2Y, accel2Z, gyro2X, gyro2Y, gyro2Z;  // IMU data for second sensor
char lastCommand[32];                                     // Last received command
unsigned int packetCount = 0;
bool telemetryEnabled = true;                             // Telemetry control
bool buzzerActivated = false;                             // Flag to track buzzer activation

// Camera stabilization variables
float cameraposition = 0;
unsigned long lastRpmTime = 0;
volatile unsigned long rpmCount = 0;
float currentInterruptTime = 0;
float timeDifference = 0;
float lastInterruptTime = 0;

// Altitude calculation variables
float apogeeAltitude = 0.0;
float maxAltitude = 0;
bool releaseActivated = false;
bool simulationMode = false;
float simulatedPressure = 0.0;
float receivedPressure = 0.0;
float referencePressure = 1013.25;
float simulatedAltitude = 0.0;
const int historySize = 10;
float altitudeHistory[historySize];
float velocityHistory[historySize];
unsigned long timestampHistory[historySize];

// PID variables
float setpoint = 1000.0;
float input = 0.0;
float output = 0.0;
float error = 0.0;
float lastError = 0.0;
float integral = 0.0;
const float K_proportional = 1.0;
const float K_integral = 0.1;
const float K_derivative = 0.01;
float heading;

// Buzzer note frequencies (Nokia Tune)
#define NOTE_CS4 277  // C#4
#define NOTE_D4  294  // D4
#define NOTE_E4  330  // E4
#define NOTE_FS4 370  // F#4
#define NOTE_GS4 415  // G#4
#define NOTE_A4  440  // A4
#define NOTE_B4  494  // B4
#define NOTE_CS5 554  // C#5
#define NOTE_D5  587  // D5
#define NOTE_E5  659  // E5

// Note durations (~130 BPM)
#define QUARTER  230
#define EIGHTH   115
#define PAUSE    50

// Nokia Tune melody and durations
int melody[] = {
  NOTE_E5, NOTE_D5, NOTE_FS4, NOTE_GS4,
  NOTE_CS5, NOTE_B4, NOTE_D4, NOTE_E4,
  NOTE_B4, NOTE_A4, NOTE_CS4, NOTE_E4,
  NOTE_A4
};

int durations[] = {
  EIGHTH, EIGHTH, QUARTER, QUARTER,
  EIGHTH, EIGHTH, QUARTER, QUARTER,
  EIGHTH, EIGHTH, QUARTER, QUARTER,
  QUARTER
};

int songLength = sizeof(melody) / sizeof(melody[0]);

/*----------------------------Functions----------------------------*/

// Play a single note using PWM
void playNote(int frequency, int duration) {
  int period = 1000000 / frequency;
  int halfPeriod = period / 2;
  long cycles = (long)duration * 1000 / period;
  for (long i = 0; i < cycles; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(halfPeriod);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(halfPeriod);
  }
  digitalWrite(BUZZER_PIN, LOW);
}

// Play the Nokia Tune
void playNokiaTune() {
  for (int i = 0; i < songLength; i++) {
    playNote(melody[i], durations[i]);
    delay(PAUSE);
  }
}

// Activate release mechanism
void activateReleaseMechanism() {
  servo.write(90);
  Serial.println("Release mechanism activated - Servo set to 90 degrees");
}

// Calculate altitude from pressure
float calculateAltitude(float pressure) {
  const float temperatureLapseRate = 0.0065;
  const float seaLevelTemperature = 288.15;
  const float gasConstant = 8.3144598;
  const float molarMass = 0.0289644;
  const float gravity = 9.80665;
  float altitude = (seaLevelTemperature / temperatureLapseRate) * (1 - pow((pressure / referencePressure), (gasConstant * temperatureLapseRate) / (gravity * molarMass)));
  return altitude;
}

// Update altitude history
void updateAltitudeHistory(float altitudeHistory[], unsigned long timestampHistory[], float newAltitude, int size) {
  for (int i = size - 1; i > 0; i--) {
    altitudeHistory[i] = altitudeHistory[i - 1];
    timestampHistory[i] = timestampHistory[i - 1];
  }
  altitudeHistory[0] = newAltitude;
  timestampHistory[0] = millis();
}

// Update velocity history
void updateVelocityHistory(float altitudeHistory[], float velocityHistory[], unsigned long timestampHistory[], int size) {
  unsigned long timeDifferenceMillis = timestampHistory[0] - timestampHistory[1];
  float timeDifferenceSeconds = timeDifferenceMillis / 1000.0;
  float latestVelocity = (altitudeHistory[0] - altitudeHistory[1]) / timeDifferenceSeconds;
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

// PID control for camera stabilization
void pidControl(float input, float setpoint, float &lastError, float &integral, Servo &servo) {
  const float K_proportional = 1.0;
  const float K_integral = 0.1;
  const float K_derivative = 0.01;
  float error = setpoint - input;
  if (error > 180) error = error - 360;
  else if (error < -180) error = error + 360;
  integral += error;
  float derivative = error - lastError;
  float output = K_proportional * error + K_integral * integral + K_derivative * derivative;
  lastError = error;
  int currentAngle = servo.read();
  int targetAngle = map(input, 0, 360, 0, 180);
  int adjustedAngle = currentAngle - (int)output;
  if (adjustedAngle < 0) adjustedAngle = 0;
  if (adjustedAngle > 180) adjustedAngle = 180;
  servo.write(targetAngle);
  if (abs(currentAngle - targetAngle) > 5) servo.write(targetAngle);
  Serial.print("Heading: ");
  Serial.print(input);
  Serial.print("°  Target Servo Angle: ");
  Serial.print(targetAngle);
  Serial.print("°  Current Servo Angle: ");
  Serial.println(currentAngle);
}

// RPM interrupt service routine
void rpmISR() {
  currentInterruptTime = millis();
  timeDifference = currentInterruptTime - lastInterruptTime;
  lastInterruptTime = currentInterruptTime;
}

// Handle incoming commands
void handleCommand(const char *command) {
  char field1[10], field2[10], field3[10], field4[10];
  int num = sscanf(command, "CMD, %*d, %9[^,], %9[^,], %9[^,], %9[^,]", field1, field2, field3, field4);
  if (num < 1) {
    Serial.println("Invalid command");
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
        Serial.println("Telemetry started.");
      } else if (strcmp(field2, "OFF") == 0) {
        telemetryEnabled = false;
        strncpy(lastCommand, "CXOFF", sizeof(lastCommand));
        Serial.println("Telemetry stopped.");
      }
      break;
    case 2:
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
    case 3:
      if (strcmp(field2, "ENABLE") == 0) {
        simulationMode = true;
        Serial.println("Simulation mode enabled.");
        strncpy(lastCommand, "SIM_ENABLE", sizeof(lastCommand));
      } else if (strcmp(field2, "ACTIVATE") == 0 && simulationMode) {
        Serial.println("Simulation activated. Waiting for pressure input...");
        strncpy(lastCommand, "SIM_ACTIVATE", sizeof(lastCommand));
        char pressureInput[16];
        while (!Serial1.available()) delay(1);
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
    case 4:
      tempPressure = atof(field2);
      if (tempPressure > 0.0) {
        simulatedPressure = tempPressure;
        Serial.println("Simulated pressure set via SIMP.");
        strncpy(lastCommand, "SIMP", sizeof(lastCommand));
      } else {
        Serial.println("Invalid pressure value in SIMP command.");
      }
      break;
    case 5:
      Serial.println("CAL command received.");
      strncpy(lastCommand, "CAL", sizeof(lastCommand));
      referencePressure = ens220.getPressureHectoPascal();
      Serial.print("Calibration complete. Reference pressure set to: ");
      Serial.println(referencePressure);
      break;
    case 6:
      if (strcmp(field2, "RELEASE") == 0) {
        if (strcmp(field3, "ON") == 0) {
          Serial.println("MEC RELEASE ON command received.");
          activateReleaseMechanism();
          strncpy(lastCommand, "MEC_RELEASE_ON", sizeof(lastCommand));
        } else if (strcmp(field3, "OFF") == 0) {
          Serial.println("MEC RELEASE OFF command received.");
          servo.write(0);
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
    default:
      Serial.println("Unknown command type or transmission error");
      break;
  }
}

// ENS220 Sensor Initialization
void SingleShotMeasure_setup() {
  Serial.println("Starting ENS220 example 01_Basic_I2C_SingleShot");
  i2c_1.begin(Wire, I2C_ADDRESS);
  while (ens220.begin(&i2c_1) != true) {
    Serial.println("Waiting for I2C to start");
    delay(1000);
  }
  Serial.print("Device UID: "); Serial.println(ens220.getUID(), HEX);
  ens220.setDefaultConfiguration();
  ens220.setPressureConversionTime(ENS220::PressureConversionTime::T_16_4);
  ens220.setOversamplingOfPressure(ENS220::Oversampling::N_32);
  ens220.setOversamplingOfTemperature(ENS220::Oversampling::N_32);
  ens220.setPressureTemperatureRatio(ENS220::PressureTemperatureRatio::PT_1);
  ens220.setStandbyTime(ENS220::StandbyTime::OneShotOperation);
  ens220.setPressureDataPath(ENS220::PressureDataPath::Direct);
  ens220.writeConfiguration();
}

void SingleShotMeasure_loop() {
  ens220.singleShotMeasure(ENS220::Sensor::TemperatureAndPressure);
  ens220.waitSingleShot();
  auto result = ens220.update();
  if (result == ENS220::Result::Ok) {
    if (hasFlag(ens220.getDataStatus(), ENS220::DataStatus::PressureReady) && hasFlag(ens220.getDataStatus(), ENS220::DataStatus::TemperatureReady)) {
      Serial.print("P[hPa]:");
      Serial.print(ens220.getPressureHectoPascal());
      Serial.print("\tT[C]:");
      Serial.println(ens220.getTempCelsius());
    }
  }
}

// Update mission time using RTC
void updateTime(char *currentTime, size_t size) {
  if (rtc.isrunning()) {
    DateTime now = rtc.now();
    snprintf(currentTime, size, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  } else {
    Serial.println("RTC not running, using fallback time.");
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

// Update flight state
void updateFlightState(float altitude, float velocity, float x, float y, float z) {
  switch (flightState) {
    case LAUNCH_PAD:
      if (altitude > 5 && velocity > 5) {
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
      if (velocity == 0 && millis() - lastOrientationTime > 10000 && x == lastOrientationX && y == lastOrientationY && z == lastOrientationZ) {
        flightState = LANDED;
        landedTime = millis();
        Serial.println("Flight state: LANDED");
      }
      break;
    case LANDED:
      break;
  }
  lastOrientationTime = millis();
  lastOrientationX = x;
  lastOrientationY = y;
  lastOrientationZ = z;
}

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.begin(2000000);  // Debugging output
  Serial1.begin(9600);    // XBee communication
  Wire.begin();
  Wire.setClock(400000);
  Serial.println("test1");

  if (!rtc.begin()) {
    Serial.println("Couldn't find DS1307 RTC!");
    while (1) delay(10);
  }
  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running, setting time to compile time.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  Serial.println("DS1307 RTC initialized.");

  pinMode(CAMERA1_PIN, OUTPUT);
  digitalWrite(CAMERA1_PIN, LOW);
  pinMode(CAMERA2_PIN, OUTPUT);
  digitalWrite(CAMERA2_PIN, LOW);

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
  } else {
    Serial.println("SD card initialized successfully... We won the game");
  }
  backupFile = SD.open("backup.txt", FILE_WRITE);

  pixels.begin();
  pixels.setPixelColor(0, 255, 0, 0);
  pixels.show();

  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmISR, RISING);

  if (!gps.begin()) {
    Serial.println("GNSS v3 initialization failed!");
  }

  if (!lis3mdl_FC.begin_I2C(MAG1_I2C_ADDRESS)) {
    Serial.println("Failed to find LIS3MDL #1");
  } else {
    Serial.println("LIS3MDL #1 Found!");
    lis3mdl_FC.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl_FC.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl_FC.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl_FC.setRange(LIS3MDL_RANGE_4_GAUSS);
  }

  if (!IMU.begin()) {
    Serial.println("Error initializing LSM6DS3 #1!");
  } else {
    Serial.print("Accelerometer #1 sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.println("Acceleration in g's");
    Serial.println("X\tY\tZ");
  }

  if (!lis3mdl.begin_I2C()) {
    Serial.println("Failed to find LIS3MDL chip");
  } else {
    Serial.println("LIS3MDL Found!");
    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  }

  SingleShotMeasure_setup();  // Initialize ENS220
  Serial.println(millis());
}

void loop() {
  Serial.println("yo");

  updateTime(currentTime, sizeof(currentTime));
  char command[64];
  if (Serial1.readBytesUntil('\n', command, sizeof(command) - 1) > 0) {
    command[sizeof(command) - 1] = '\0';
    handleCommand(command);
  }
  Serial.println("yo2");
  unsigned long missionTime = millis() / 1000;

  float rpm = (timeDifference > 0) ? (60000 / timeDifference) : 0;
  lastRpmTime = millis();
  rpmCount = 0;

  float currentVoltage = analogRead(BATTERY_PIN) * voltageDividerFactor;

  float latitude = 0.0, longitude = 0.0, gpsAltitude = 0.0;
  unsigned int satellites = 0;
  if (gps.getFixType() >= 3) {
    latitude = gps.getLatitude() / 10000000.0;
    longitude = gps.getLongitude() / 10000000.0;
    gpsAltitude = gps.getAltitude() / 1000.0;
    satellites = gps.getSIV();
    snprintf(gpsTime, sizeof(gpsTime), "%02d:%02d:%02d", gps.getHour(), gps.getMinute(), gps.getSecond());
  }
  Serial.println("yo3");

  sensors_event_t magEvent1;
  lis3mdl.getEvent(&magEvent1);
  sensors_event_t magEvent2;
  lis3mdl_CAM.getEvent(&magEvent2);
  mag2X = magEvent2.magnetic.x;
  mag2Y = magEvent2.magnetic.y;
  mag2Z = magEvent2.magnetic.z;

  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    Serial.print(accelX);
    Serial.print('\t');
    Serial.print(accelY);
    Serial.print('\t');
    Serial.println(accelZ);
  }
  Serial.println("yo4");
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
  }
  Serial.println("yo5");

  SingleShotMeasure_loop();
  float temperature = ens220.getTempCelsius();
  float pressure = ens220.getPressureHectoPascal();

  float altitude = calculateAltitude(pressure);
  if (simulationMode) {
    simulatedAltitude = (1 - pow(simulatedPressure / 1013.25, 0.190284)) * 145366.45;
    altitude = simulatedAltitude;
  }
  Serial.println("yo6");

  if (flightState == ASCENT && altitude > maxAltitude) {
    maxAltitude = altitude;
  }

  if (lastTransmissionTime + 1000 < millis()) {
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
      Serial.println("yo7");
      int altitude_int = (int)(altitude * 10);
      int temperature_int = (int)(temperature * 10);
      int pressure_int = (int)(pressure * 10);
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
      int gpsAltitude_int = (int)(gpsAltitude * 10);
      int latitude_int = (int)(latitude * 10000);
      int longitude_int = (int)(longitude * 10000);
      snprintf(telemetry, sizeof(telemetry),
               "%s,%s,%u,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%d,%d,%d,%u,%s,COSMOS",
               TEAM_ID, currentTime, packetCount, mode, state,
               altitude_int, temperature_int, pressure_int, currentVoltage_int,
               gyroX_int, gyroY_int, gyroZ_int, accelX_int, accelY_int, accelZ_int,
               magX_int, magY_int, magZ_int, rpm_int, gpsTime, gpsAltitude_int,
               latitude_int, longitude_int, satellites, lastCommand);
      Serial1.println(telemetry);
      Serial.println(telemetry);
      dataFile = SD.open("data.txt", FILE_WRITE);
      if (dataFile) {
        dataFile.println(telemetry);
        dataFile.close();
        Serial.println("Finished writing to SD card!");
      } else {
        Serial.println("Error writing to SD card!.... We lost the game");
      }
      Serial.println("yo8");
      packetCount++;
    }
  }

  heading = atan2(magEvent2.magnetic.y, magEvent2.magnetic.x);
  heading = heading * 180 / PI;
  if (heading < 0) heading += 360;

  updateAltitudeHistory(altitudeHistory, timestampHistory, altitude, historySize);
  updateVelocityHistory(altitudeHistory, velocityHistory, timestampHistory, historySize);

  switch (flightState) {
    case LAUNCH_PAD:
      buzzerActivated = false;
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      break;
    case ASCENT:
      buzzerActivated = false;
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      break;
    case APOGEE:
      buzzerActivated = false;
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      break;
    case DESCENT:
      buzzerActivated = false;
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      if (!releaseActivated && altitude <= (apogeeAltitude * 0.75)) {
        activateReleaseMechanism();
        releaseActivated = true;
      }
      pidControl(heading, setpoint, lastError, integral, servo);
      break;
    case LANDED:
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      if (!buzzerActivated) {
        Serial.println("Flight landed - Activating buzzer with Nokia Tune");
        playNokiaTune();
        buzzerActivated = true;
      }
      break;
  }
}
