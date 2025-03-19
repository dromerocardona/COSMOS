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
#include "FeedBackServo.h"
#include <Adafruit_NeoPixel.h>
#include <Servo.h>


// PINS AND DEFINITIONS
#define BATTERY_PIN A0 // Analog pin for voltage divider circuit
#define RPM_PIN 2      // Pin for Hall effect sensor
#define SD_CS_PIN 6    // Chip select pin for SD card
#define SERVO_PIN 13    // Servo pin for GND camera stabilization
#define FEEDBACK_PIN 9 // Feedback signal pin for servo control
#define I2C_ADDRESS 0x20 // I2C Address for ENS 220
#define SERIAL_BAUDRATE 57600 // Speed of Serial Communication with the computer (ENS220)
#define INTN_1 4 // Interrupt pin for ENS220
#define CAMERA1_PIN 10 // Blade camera
#define CAMERA2_PIN 11 // Ground camera
#define LED_DATA 5
#define NUM_LEDS 5 // Number of LEDs for FastLED
#define MAG1_I2C_ADDRESS 0x1C  // First LIS3MDL address
#define MAG2_I2C_ADDRESS 0x1E  // Second LIS3MDL address
#define IMU1_I2C_ADDRESS 0x6A  // First LSM6DS3 address
#define IMU2_I2C_ADDRESS 0x6B  // Second LSM6DS3 address

Adafruit_NeoPixel pixels(NUM_LEDS, LED_DATA, NEO_GRB + NEO_KHZ800);

// Team ID
#define TEAM_ID "3195"

/*-----STATE MANAGEMENT VARIABLE-----*/
enum FlightState {
  LAUNCH_PAD,
  ASCENT,
  APOGEE,
  DESCENT,
  LANDED
};

//use to change state
FlightState flightState = LAUNCH_PAD; // Initial state
// call with currentState

// Sensor objects
ScioSense::ENS220 ens220;
Adafruit_LIS3MDL lis3mdl;// Magnetometer
Adafruit_LIS3MDL lis3mdl_FC;  // First magnetometer
Adafruit_LIS3MDL lis3mdl_CAM;  // Second magnetometer
float apogeeAltitude = 0.0;
unsigned long landedTime = 0;
unsigned long lastOrientationTime = 0;
float lastOrientationX = 0.0, lastOrientationY = 0.0, lastOrientationZ = 0.0;
SFE_UBLOX_GNSS gps;
uint8_t satellites = 0;
Servo servo;

// Variables
float voltageDividerFactor = 0.012089; // Adjust based on resistor values in voltage divider
unsigned int packetCount = 0;
File dataFile;
bool telemetryEnabled = false;  // Telemetry Control
float lastTransmissionTime = 0; // Last time of telemetry transmission
char currentTime[9] = "00:00:00"; // Mission time in "HH:MM:SS"
char lastCommand[32] = ""; // Last received command
char gpsTime[9] = "00:00:00"; // GPS time in "HH:MM:SS"

// Magnetometer data for second sensor
float mag2X, mag2Y, mag2Z;

// IMU data for second sensor (if separate IMU object is used)
float accel2X, accel2Y, accel2Z;
float gyro2X, gyro2Y, gyro2Z;

// Camera stabilization variables
float cameraposition = 0;
unsigned long lastRpmTime = 0; // last time of an magnet detection
volatile unsigned long rpmCount = 0; // RPM counter
float currentInterruptTime = 0; // Current time of an interrupt
float timeDifference = 0; // Time difference between two consecutive interrupts
float lastInterruptTime = 0;

// Simulation mode variables
// Global variables
bool simulationMode = false;
float simulatedPressure = 0.0;
float receivedPressure = 0.0; // For SIM_ACTIVATE pressure input
float referencePressure = 1013.25; // Default reference point (sea level)


float simulatedAltitude = 0.0;      // Altitude derived from simulated pressure

const int historySize = 10; // Fixed size of the history arrays
float altitudeHistory[historySize];
float velocityHistory[historySize];
unsigned long timestampHistory[historySize]; // To store time in milliseconds

// Function to calculate altitude from pressure
float calculateAltitude(float pressure) {
    // Constants for the barometric formula
    const float temperatureLapseRate = 0.0065; // Temperature lapse rate in K/m
    const float seaLevelTemperature = 288.15; // Sea level standard temperature in K
    const float gasConstant = 8.3144598; // Universal gas constant in J/(mol*K)
    const float molarMass = 0.0289644; // Molar mass of Earth's air in kg/mol
    const float gravity = 9.80665; // Acceleration due to gravity in m/s^2

    // Calculate altitude using the barometric formula
    float altitude = (seaLevelTemperature / temperatureLapseRate) * 
                     (1 - pow((pressure / referencePressure), (gasConstant * temperatureLapseRate) / (gravity * molarMass)));
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
    float timeDifferenceSeconds = timeDifferenceMillis / 1000.0; // Convert to seconds

    // Calculate the latest velocity
    float latestVelocity = (altitudeHistory[0] - altitudeHistory[1]) / timeDifferenceSeconds;

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

// PID tuning parameters
const float K_proportional = 1.0; // Proportional gain
const float K_integral = 0.1; // Integral gain
const float K_derivative = 0.01; // Derivative gain
/*EXPLANATION FOR TUNING REFRENCE:
"P" accounts for large error in the system an determines how fast errors are corrected (values above 2 are usually too high)
"I" helps with drift and lag in the system (the value for this is generaly small like 0.1 or 0.05)
"D" accounts for rapid changed to stabilize the system and improve response by dampening occilations (values are determined by system response time)

General guide for PID tuning:
-Tune in order of "P", "D", "I"
  -Start with "P", increase until the system STARTS to oscillate (unconverging) (I would guess between 1.2 and 1.8)
  -Increase "D" until the system is stable (does not overshoot)
  -Finally, increase "I" until the system is stable and acurate 
*/

// Variables used for PID
float setpoint = 1000.0; // Desired setpoint placeholder
float input = 0.0; // Current system input
float output = 0.0; // PID output
float error = 0.0; // Current error
float lastError = 0.0; // Previous error
float integral = 0.0; // tracks cumulative error

// Funtion for PID Camera Stabalization
void pidControl(float input, float setpoint, float &lastError, float &integral, Servo &servo) {
    // PID tuning parameters
    const float K_proportional = 1.0; // Proportional gain
    const float K_integral = 0.1; // Integral gain
    const float K_derivative = 0.01; // Derivative gain

    // Calculate the error
    float error = setpoint - input; // Calculate the error based on difference between setpoint and input
    if (error > 180) {
        error = error - 360; // account for the closest path to the setpoint being across the 0° line
    } else if (error < -180) {
        error = error + 360; // account for the closest path to the setpoint being across the 0° line the other way
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
    int currentAngle = servo.read(); // Read current servo position
    int targetAngle = map(input, 0, 360, 0, 180); // Adjust based on servo range
    int adjustedAngle = currentAngle - (int)output; // Adjusting servo based on the PID output

    // Constrain to servo range (0-180°)
    if (adjustedAngle < 0) adjustedAngle = 0;
    if (adjustedAngle > 180) adjustedAngle = 180;
    servo.write(targetAngle); // Set servo to target angle

    // Read feedback and correct (optional, depends on FeedBackServo implementation)
    if (abs(currentAngle - targetAngle) > 5) { // Tolerance of 5 degrees
        servo.write(targetAngle); // Reapply correction
    }

    // Debug output
    Serial.print("Heading: ");
    Serial.print(input);
    Serial.print("°  Target Servo Angle: ");
    Serial.print(targetAngle);
    Serial.print("°  Current Servo Angle: ");
    Serial.println(currentAngle);
}

// Interrupt Service Routine for RPM counting
void rpmISR() {
  currentInterruptTime = millis();// Get the current time
  timeDifference = currentInterruptTime - lastInterruptTime;  // Calculate the time difference between interrupts
  lastInterruptTime = currentInterruptTime;
  
  // |  We could potentialy calculate the instantaneous RPM in break instead of in loop
  // V
  // instantaneousRPM = 60000.0 / timeDifference;
}

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
      if (velocity == 0 && millis() - lastOrientationTime > 10000 && 
        x == lastOrientationX && y == lastOrientationY && z == lastOrientationZ) {
        flightState = LANDED;
        landedTime = millis();
        Serial.println("Flight state: LANDED");
      }
    break;
    case LANDED:
      {}
    break;
  }
  // Update last orientation values
    lastOrientationTime = millis();
    lastOrientationX = x;
    lastOrientationY = y;
    lastOrientationZ = z;
}

// Function to handle commands
void handleCommand(const char* command) {
    char field1[10], field2[10], field3[10], field4[10];
    int num = sscanf(command, "CMD, %*d, %9[^,], %9[^,], %9[^,], %9[^,]", field1, field2, field3, field4);

    // Check if parsing failed
    if (num < 1) {
        Serial.println("Invalid command");
        return;
    }

    // Handle CX commands
    if (strcmp(field1, "CX") == 0 && num >= 2) {
        if (strcmp(field2, "ON") == 0) {
            telemetryEnabled = true;
            Serial.println("Telemetry started.");
        } else if (strcmp(field2, "OFF") == 0) {
            telemetryEnabled = false;
            Serial.println("Telemetry stopped.");
        }
    }
    // Handle ST commands
    else if (strcmp(field1, "ST") == 0 && num >= 2) {
        if (strcmp(field2, "UTC_TIME") == 0) {
            Serial.println("ST UTC_TIME command received.");
            // Add specific UTC_TIME action here if needed
        } else if (strcmp(field2, "GPS") == 0) {
            Serial.println("ST GPS command received.");
            // Add specific GPS action here if needed
        }
    }
    // Handle SIM commands
    else if (strcmp(field1, "SIM") == 0 && num >= 2) {
        if (strcmp(field2, "ENABLE") == 0) {
            simulationMode = true;
            Serial.println("Simulation mode enabled.");
        } else if (strcmp(field2, "ACTIVATE") == 0) {
            if (simulationMode) {
                Serial.println("Simulation activated. Waiting for pressure input...");
                char pressureInput[16];
                while (!Serial1.available()) {
                    delay(1); // Wait for data from Serial1 (e.g., XBee)
                }
                Serial1.readBytesUntil('\n', pressureInput, sizeof(pressureInput) - 1);
                pressureInput[sizeof(pressureInput) - 1] = '\0';
                receivedPressure = atof(pressureInput);
                if (receivedPressure > 0.0) {
                    simulatedPressure = receivedPressure;
                    Serial.println("Simulated pressure updated.");
                } else {
                    Serial.println("Invalid pressure value received. Using default pressure.");
                }
            } else {
                Serial.println("Simulation mode not enabled yet.");
            }
        } else if (strcmp(field2, "DISABLE") == 0) {
            simulationMode = false;
            Serial.println("Simulation mode disabled.");
        }
    }
    // Handle SIMP command
    else if (strcmp(field1, "SIMP") == 0 && num >= 2) {
        float pressure = atof(field2);
        if (pressure > 0.0) {
            simulatedPressure = pressure;
            Serial.println("Simulated pressure set via SIMP.");
        } else {
            Serial.println("Invalid pressure value in SIMP command.");
        }
    }
    // Handle CAL command
    else if (strcmp(field1, "CAL") == 0) {
        Serial.println("CAL command received.");
        referencePressure = ens220.getPressureHectoPascal();
        Serial.print("Calibration complete. Reference pressure set to: ");
        Serial.println(referencePressure);
    }
    // Handle MEC commands
    else if (strcmp(field1, "MEC") == 0 && num >= 2) {
        if (strcmp(field2, "RELEASE") == 0 && num >= 3) {
            if (strcmp(field3, "ON") == 0) {
                Serial.println("MEC RELEASE ON command received.");
                // Add release ON action here
            } else if (strcmp(field3, "OFF") == 0) {
                Serial.println("MEC RELEASE OFF command received.");
                // Add release OFF action here
            }
        } else if (strcmp(field2, "CAMERA") == 0 && num >= 3) {
            if (strcmp(field3, "BLADE") == 0 && num >= 4) {
                if (strcmp(field4, "ON") == 0) {
                    digitalWrite(CAMERA1_PIN, HIGH); // Using CAMERA_PIN for simplicity
                    Serial.println("MEC CAMERA BLADE ON - Camera powered ON.");
                } else if (strcmp(field4, "OFF") == 0) {
                    digitalWrite(CAMERA1_PIN, LOW);
                    Serial.println("MEC CAMERA BLADE OFF - Camera powered OFF.");
                }
            } else if (strcmp(field3, "GROUND") == 0 && num >= 4) {
                if (strcmp(field4, "ON") == 0) {
                    digitalWrite(CAMERA2_PIN, HIGH); // Using CAMERA_PIN for simplicity
                    Serial.println("MEC CAMERA GROUND ON - Camera powered ON.");
                } else if (strcmp(field4, "OFF") == 0) {
                    digitalWrite(CAMERA2_PIN, LOW);
                    Serial.println("MEC CAMERA GROUND OFF - Camera powered OFF.");
                }
            } else if (strcmp(field3, "STABLE") == 0 && num >= 4) {
              Serial.println("MEC CAMERA STABLE command received.");
              // PID Camera Stabalization
              pidControl(heading, setpoint, lastError, integral, servo);
            }
        }
    }
    // Unknown command type
    else {
        Serial.println("Unknown command type or transmission error");
    }
}
// USAGE!!!!
// Define the command as a modifiable C-style string (array of characters)
char command[] = "CMD, 3195, CX, ON";
// handleCommand(command); // Pass the command to the function

// ENS220 Sensor Initialization
void ContinuousModeWithFIFO_setup() {
Serial.println("Starting ENS220 example 03_FIFO_I2C_Continuous");

// Start the communication, confirm the device PART_ID, and read the device UID
I2cInterface i2c_1;
i2c_1.begin(Wire, I2C_ADDRESS);
while (ens220.begin(&i2c_1) != true)
{
  Serial.println("Waiting for I2C to start");
  delay(1000);
}
Serial.print("Device UID: "); Serial.println(ens220.getUID(), HEX);

// Set up ENS220 configuration
ens220.setDefaultConfiguration();
ens220.setPressureConversionTime(ENS220::PressureConversionTime::T_16_4);
ens220.setOversamplingOfPressure(ENS220::Oversampling::N_16);
ens220.setOversamplingOfTemperature(ENS220::Oversampling::N_16);
ens220.setPressureTemperatureRatio(ENS220::PressureTemperatureRatio::PT_32);        
ens220.setPressureDataPath(ENS220::PressureDataPath::Fifo);
ens220.setInterfaceConfiguration(ENS220::InterfaceConfiguration::InterruptEnable);
ens220.setInterruptConfiguration(ENS220::InterruptConfiguration::TemperatureDataReady | ENS220::InterruptConfiguration::PressureFifoFull); 
ens220.setInterruptPin(INTN_1);

ens220.writeConfiguration();
ens220.startContinuousMeasure(ENS220::Sensor::TemperatureAndPressure);
}

void ContinuousModeWithFIFO_loop(){    
  // Poll the interrupt pin until a new value is available
  ens220.waitInterrupt();

  // Check the DATA_STAT from the sensor. If data is available, it reads it
  auto result= ens220.update();
  if(result == ENS220::Result::Ok)
  {      
    if(hasFlag(ens220.getInterruptStatus(), ENS220::InterruptStatus::FifoFull))
    {
      for(int i=0; i<32; i++)
      {
        // Send the pressure value that was collected during the ens220.update()
        Serial.print("P[hPa]:");
        Serial.println(ens220.getPressureHectoPascal(i));
      }
    }
    if(hasFlag(ens220.getInterruptStatus(), ENS220::InterruptStatus::Temperature))
    {
      // Send the temperature value that was collected during the ens220.update()
      Serial.print("T[C]:");
      Serial.println(ens220.getTempCelsius());
    }
  }
}

void updateTime(char *currentTime, size_t size) {
  static unsigned long previousMillis = 0;
  const unsigned long interval = 1000; // 1 second interval
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Extract hours, minutes, seconds from string
    int hours, minutes, seconds;
    if (sscanf(currentTime, "%2d:%2d:%2d", &hours, &minutes, &seconds) == 3) {
      // Increment time
      seconds++;
      if (seconds >= 60) {
        seconds = 0;
        minutes++;
        if (minutes >= 60) {
          minutes = 0;
          hours++;
          if (hours >= 24) {
            hours = 0;
          }
        }
      }
      // Convert back to string with leading zeros
      snprintf(currentTime, size, "%02d:%02d:%02d", hours, minutes, seconds);
    }
  }
}

void setup() {
  Serial.begin(115200);         // Debugging output
  Serial1.begin(9600);          // XBee communication
  Wire.begin();

  // Initialize camera 1 control pin (e.g., for power on/off)
  pinMode(CAMERA1_PIN, OUTPUT);  // Set camera control pin to output
  digitalWrite(CAMERA1_PIN, LOW);  // Make sure camera is OFF initially

  // Camera 2 control
  pinMode(CAMERA2_PIN, OUTPUT);  // Set camera control pin to output
  digitalWrite(CAMERA2_PIN, LOW);  // Make sure camera is OFF initially

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
  } else {
    Serial.println("SD card initialized successfully");
  }
  
  // NeoPixel initialization
  pixels.setPixelColor(0, 255, 0, 0);  // Set first pixel to red
  pixels.show();  // Update the pixels

  // Initialize RPM sensor
  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmISR, RISING);

  // Initialize GPS (using Serial1 to communicate with GPS)
  Serial1.begin(9600);

  // Initialize GNSS v3 (using Serial1 to communicate with GPS)
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

  // Initialize first IMU (LSM6DS3)
  if (!IMU.begin()) {
    Serial.println("Error initializing LSM6DS3 #1!");
  } else {
    Serial.print("Accelerometer #1 sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");

  // Initialize LIS3MDL (Magnetometer)
  if (!lis3mdl.begin_I2C()) {          // hardware I2C mode
    Serial.println("Failed to find LIS3MDL chip");
  } else {
    Serial.println("LIS3MDL Found!");
  }

  // Set up LIS3MDL settings
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  pixels.begin();

  // Initialize variables
  //lastRpmTime = millis();
}

/*____________________________________________________________________________________________________*/
/*________________________________________________MAIN________________________________________________*/
/*____________________________________________________________________________________________________*/
void loop() {

  /*------------------CORE OPERATIONS-----------------------------------------------------------*/
  //these run every loop regardless of state
  updateTime(currentTime, sizeof(currentTime));

  /*------------------CORE OPERATIONS----Read Comands-------------------------------------------*/
  // Read data from XBee or Serial1
  char command[64];
  if (Serial1.readBytesUntil('\n', command, sizeof(command) - 1) > 0) {
    command[sizeof(command) - 1] = '\0'; // Ensure null-termination
    handleCommand(command);  // Process the command
    strncpy(lastCommand, command, sizeof(lastCommand) - 1);
    lastCommand[sizeof(lastCommand) - 1] = '\0';
  }

  /*------------------CORE OPERATIONS----Sensor Data--------------------------------------------*/
  //potentioal consideration for PID stability later:
  //the MOST TIME SENSITIVE OPERATIONS go first, things like altitude can wait and not be updated
  //on every loop iteration to prioritize accel/gyro/mag and timer updates
  //  this could be accomplished by having a tenth second timer or something similar

  unsigned long missionTime = millis() / 1000; // Mission time in seconds

  // Calculate instantaneous RPM
  float rpm = (timeDifference > 0) ? (60000 / timeDifference) : 0; // Calculate RPM
  lastRpmTime = millis();  // Imediately update last RPM time for min error
  rpmCount = 0;

  // Read battery voltage
  float currentVoltage = analogRead(BATTERY_PIN) * voltageDividerFactor;

  // Read GPS data
  float latitude = 0.0, longitude = 0.0, gpsAltitude = 0.0;
  unsigned int satellites = 0;
  
  if (gps.getFixType() >= 3) { // Check if we have a 3D fix
      latitude = gps.getLatitude() / 10000000.0; // Convert to degrees
      longitude = gps.getLongitude() / 10000000.0; // Convert to degrees
      gpsAltitude = gps.getAltitude() / 1000.0; // Convert to meters
      satellites = gps.getSIV();
      snprintf(gpsTime, sizeof(gpsTime), "%02d:%02d:%02d", gps.getHour(), gps.getMinute(), gps.getSecond());
  }
  
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
    Serial.print(accelX);
    Serial.print('\t');
    Serial.print(accelY);
    Serial.print('\t');
    Serial.println(accelZ);
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
  }
  
  // Retrieve Temperature and Pressure from ENS220
  float temperature = ens220.getTempCelsius();
  float pressure = ens220.getPressureHectoPascal();

  // Calculate altitude from pressure
  float altitude = calculateAltitude(pressure);

  // ENS220 Continuous mode data reading
  ContinuousModeWithFIFO_loop();

  /*------------------CORE OPERATIONS----Sensor Data----altitude and simulation mode-----------*/
  // Use simulated pressure if in simulation mode
  if (simulationMode) {
    simulatedAltitude = (1 - pow(simulatedPressure / 1013.25, 0.190284)) * 145366.45;  // Approximation formula
    altitude = simulatedAltitude;
  }

  /*------------------TELEMETRY TRASMISSION----------------------------------------------------*/
  // Telemetry Transmission
  if(lastTransmissionTime+1000<millis()) {// delay wraper:Transmit telemetry every second
    lastTransmissionTime = millis();
    if (telemetryEnabled) {
      char telemetry[256];
      const char* mode = simulationMode ? "S" : "F";
      const char* state;
      switch (flightState) {
        case LAUNCH_PAD: state = "LAUNCH_PAD"; break;
        case ASCENT: state = "ASCENT"; break;
        case APOGEE: state = "APOGEE"; break;
        case DESCENT: state = "DESCENT"; break;
        case LANDED: state = "LANDED"; break;
        default: state = "UNKNOWN"; break;
      }
      snprintf(telemetry, sizeof(telemetry),
               "%s,%s,%u,%s,%s,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%s,%.1f,%.1f,%.4f,%.4f,%u,%s,COSMOS",
               TEAM_ID, currentTime, packetCount, mode, state, altitude, temperature, pressure, currentVoltage,
               gyroX, gyroY, gyroZ, accelX, accelY, accelZ,
               magEvent1.magnetic.x, magEvent1.magnetic.y, magEvent1.magnetic.z, rpm, gpsTime, gpsAltitude,
               latitude, longitude, satellites, lastCommand);

      Serial1.println(telemetry);

      // Save telemetry to SD card
      dataFile = SD.open("telemetry.txt", FILE_WRITE);
      if (dataFile) {
          dataFile.println(telemetry);
          dataFile.close();
      } else {
          Serial.println("Error writing to SD card!");
      }

      packetCount++; // Increment packet count
    }
  }

  /*------------------CALCULATIONS-------------------------------------------------------------*/
  // Calculate heading
  float heading = atan2(magEvent2.magnetic.y, magEvent2.magnetic.x);
  heading = heading * 180 / PI; // Convert to degrees
  if (heading < 0) heading += 360; // Ensure 0-360 range

  //used for some state trasitions
  updateAltitudeHistory(altitudeHistory, timestampHistory, altitude, historySize);
  updateVelocityHistory(altitudeHistory, velocityHistory, timestampHistory, historySize);
  /*---------------------------------STATE DEPENDENT OPERATIONS---------------------------------*/

  //it is very important to fill this with the correct logic for trasitions and contents for each state ex. PID control in separated and deployed
  switch (flightState) {//switch statement so we have smooth-looking code :)
  ////////////////////////////////////////////////////////////////////////
    case LAUNCH_PAD:
      // Code for launch state
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      break;
  ////////////////////////////////////////////////////////////////////////
    case ASCENT:
      // Code for ascent state
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      break;
  ////////////////////////////////////////////////////////////////////////
    case APOGEE:
      // Code for ascent state
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
    break;
  ////////////////////////////////////////////////////////////////////////
    case DESCENT:
      // Code for separated state
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      // PID Camera Stabilization
      pidControl(heading, setpoint, lastError, integral, servo);
      break;
  ////////////////////////////////////////////////////////////////////////
    case LANDED:
      updateFlightState(altitude, velocityHistory[0], accelX, accelY, accelZ);
      if (avg(velocityHistory, historySize) < 1) {
        flightState = LANDED;
      }
      break;
  ////////////////////////////////////////////////////////////////////////
  }
}
/*-------------------------------------------REPEAT-------------------------------------------*/
