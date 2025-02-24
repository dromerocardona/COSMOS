// LIBRARY INCLUSIONS
#include <Arduino_LSM6DSOX.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <ScioSense_ENS220.h>
#include <ens220.h>
#include <utils.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <Arduino.h>
#include "i2c_interface.h"
#include "FeedBackServo.h"

// PINS AND DEFINITIONS
#define BATTERY_PIN A0 // Analog pin for voltage divider circuit
#define RPM_PIN 2      // Pin for Hall effect sensor
#define SD_CS_PIN 4    // Chip select pin for SD card
// #define SERVO_PIN 3   // Servo pin for camera stabilization
#define I2C_ADDRESS 0x20 // I2C Address for ENS 220
#define SERIAL_BAUDRATE 57600 // Speed of Serial Communication with the computer (ENS220)
#define INTN_1 2 // Interrupt pin for ENS220
#define CAMERA_PIN 7 // RunCam
// #define FEEDBACK_PIN 2 // Feedback signal pin for servo control

// Team ID
#define TEAM_ID "3195"


/*DIEGO'S NOTES:
This is what the telemetry should ultimately look like:
--------------------------------------------------------------------------------------------------------------------------------------
String telemetry = String(TEAM_ID) + "," + STRING(missionTime) + "," STRING(packetCount) + "," + STRING(mode) + "," + STRING(state) +
"," + STRING(altitude) + "," + STRING(temperature) + "," STRING(pressure) + "," + STRING(currentVoltage) + "," +
STRING(gyroEvent.gyro.x) + "," + STRING(gyroEvent.gyro.y) + "," + STRING(gyroEvent.gyro.z) + "," + STRING(accelEvent.acceleration.x) +
"," + STRING(accelEvent.acceleration.y) + "," + STRING(accelEvent.acceleration.z) + "," + STRING(magEvent.magnetic.x) + "," +
STRING(magEvent.magnetic.y) + "," + STRING(magEvent.magnetic.z) + "," + STRING(autogyroRotationRate) + "," + STRING(gpsTime) +
"," + STRING(latitude) + "," + STRING(longitude) + "," + STRING(satellites) + "," + STRING(lastCommand) + String("COSMOS");
--------------------------------------------------------------------------------------------------------------------------------------
Notes:
all telemetry must be sent as a string
please look at the mission guide for examples of mode and state, follow them exactly
altitude must be calculated using pressure sensor (why it is calibrated)
also get gpsTime, the ground station will either set the time to the time on the gps or the computer time
for the lastCommand variable, return only a simplified version (e.g. CXON, CAM_ON, etc.)
*/

// Sensor objects
using namespace ScioSense; // ENS220
ENS220 ens220; // sensor object ENS220
TinyGPSPlus gps; // GPS sensor
Adafruit_LIS3MDL lis3mdl;// Magnetometer
// Set feedback signal pin number for the servo
// FeedBackServo servo = FeedBackServo(FEEDBACK_PIN);

// Variables
float voltageDividerFactor = 5.0; // Adjust based on resistor values in voltage divider
unsigned int packetCount = 0;
File dataFile;
bool telemetryEnabled = false;  // Telemetry Control
// Camera stabilization variables
float cameraposition = 0;
unsigned long lastRpmTime = 0; // last time of an magnet detection
volatile unsigned long rpmCount = 0; // RPM counter
float currentInterruptTime = 0; // Current time of an interrupt
float timeDifference = 0; // Time difference between two consecutive interrupts
float lastInterruptTime = 0;

// Simulation mode variables
bool simulationMode = false;
float simulatedPressure = 1013.25;  // Default sea level pressure in hPa
float simulatedAltitude = 0.0;      // Altitude derived from simulated pressure

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

// Interrupt Service Routine for RPM counting
void rpmISR() {
  currentInterruptTime = millis();// Get the current time
  timeDifference = currentInterruptTime - lastInterruptTime;  // Calculate the time difference between interrupts
  lastInterruptTime = currentInterruptTime;
  
  // |  We could potentialy calculate the instantaneous RPM in break instead of in loop
  // V
  // instantaneousRPM = 60000.0 / timeDifference;
}

// Variables
float receivedPressure = 0.0;  // Variable to store received pressure value from ground station

void handleCommand(String command) {
command.trim();  // Remove any leading or trailing spaces

if (command == "SIM_ENABLE") {
  simulationMode = true;
  Serial.println("Simulation mode enabled.");
} else if (command == "SIM_ACTIVATE") {
  if (simulationMode) {
    Serial.println("Simulation activated. Waiting for pressure input...");
    // Wait for the ground station to send pressure data via XBee
    while (!Serial1.available()) {
      // Keep waiting for data
      delay(100); 
    }
    String pressureInput = Serial1.readStringUntil('\n');  // Read the pressure string from ground station
    pressureInput.trim();  // Clean any trailing/leading spaces
    receivedPressure = pressureInput.toFloat();  // Convert the string to a float
    if (receivedPressure > 0.0) {
      simulatedPressure = receivedPressure;  // Set simulated pressure
      Serial.println("Simulated pressure updated.");
    } else {
      Serial.println("Invalid pressure value received. Using default pressure.");
    }
  } else {
    Serial.println("Simulation mode not enabled yet.");
  }
} 
else if (command == "CAMERA_ON") {
  digitalWrite(CAMERA_PIN, HIGH); // Turn camera ON
  Serial.println("Camera powered ON.");
} 
else if (command == "CAMERA_OFF") {
  digitalWrite(CAMERA_PIN, LOW);  // Turn camera OFF
  Serial.println("Camera powered OFF.");
} 
else if (command == "CX_ON") {
  telemetryEnabled = true;  // Start telemetry
  Serial.println("Telemetry started.");
}
else if (command == "CX_OFF") {
  telemetryEnabled = false; // Stop telemetry
  Serial.println("Telemetry stopped.");
}
}

// ENS220 Sensor Initialization
void ContinuousModeWithFIFO_setup()


{
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


void ContinuousModeWithFIFO_loop()
{    
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

void setup()
{
  Serial.begin(115200);         // Debugging output
  Serial1.begin(9600);          // XBee communication
  Wire.begin();

  // Initialize camera control pin (e.g., for power on/off)
  pinMode(CAMERA_PIN, OUTPUT);  // Set camera control pin to output
  digitalWrite(CAMERA_PIN, LOW);  // Make sure camera is OFF initially

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }

  // Initialize RPM sensor
  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmISR, RISING);

  // Initialize GPS (using Serial1 to communicate with GPS)
  Serial1.begin(9600);

  // Initialize LSM6DSO32 (Accelerometer and Gyroscope)
  if (!IMU.begin()) {  // Corrected reference to the lsm6dso32 object
    Serial.println("Error initializing LSM6DSOX!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");

  // Initialize LIS3MDL (Magnetometer)
  if (! lis3mdl.begin_I2C()) {          // hardware I2C mode,
    Serial.println("Failed to find LIS3MDL chip");
    while (1) { delay(10); }
  }
  Serial.println("LIS3MDL Found!");

  // Set up LIS3MDL settings
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  //servo.setServoControl(SERVO_PIN);
  //servo.setKp(1.0);  // You can adjust the PID controller gain
}


// Initialize ENS220 sensor
#ifdef DEBUG_ENS220;
ens220.enableDebugging(Serial);
#endif

// Initialize variables
//lastRpmTime = millis();

void loop() {
    // Read data from XBee or Serial1
    if (Serial1.available()) {
      String command = Serial1.readStringUntil('\n');  // Read command from XBee
      handleCommand(command);  // Process the command
    }

    unsigned long missionTime = millis() / 1000; // Mission time in seconds

    // Read battery voltage
    float currentVoltage = analogRead(BATTERY_PIN) * (5.0 / 1023.0) * voltageDividerFactor;

  // Calculate instantaneous RPM
  float rpm = (60000 / timeDifference); // Calculate RPM
  lastRpmTime = millis();  // Imediately update last RPM time for min error
  rpmCount = 0; 
  
  // Read GPS data
  float latitude = 0.0, longitude = 0.0, gpsAltitude = 0.0;
  unsigned int satellites = 0;
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }
  if (gps.altitude.isValid()) {
    gpsAltitude = gps.altitude.meters();
  }
  if (gps.satellites.isValid()) {
    satellites = gps.satellites.value();
  }

    // Use simulated pressure if in simulation mode
    if (simulationMode) {
      simulatedAltitude = (1 - pow(simulatedPressure / 1013.25, 0.190284)) * 145366.45;  // Approximation formula
      gpsAltitude = simulatedAltitude;
    }

    // Read magnetometer data
    sensors_event_t magEvent;
    lis3mdl.getEvent(&magEvent);

    // Calculate heading
    float heading = atan2(magEvent.magnetic.y, magEvent.magnetic.x);
    heading = heading * 180 / PI; // Convert to degrees
    if (heading < 0) heading += 360; // Ensure 0-360 range

  /* THIS IS THE PID CONTROL

  // get input from the system
  input = heading;

  // Calculate the error
  error = setpoint - input; // Calculate the error based on difference between setpoint and input
    //this method of error calculation may need to be changed based on the system

  // Calculate the integral term
  integral += error;

  // Calculate the derivative term
  float derivative = error - lastError;

  // Calculate the PID output
  output = K_proportional * error + K_integral * integral + K_derivative * derivative;

  // Update the system based on the PID output
  updateSystem(output);

  // Store the current error as the last error to prepare for the next iteration
  lastError = error;

  // Print the current values for debugging
  //Serial.print("Input: ");
  //Serial.print(input);
  //Serial.print(", Output: ");
  //Serial.println(output);

  *///END OF PID CONTROL

    // Serial.print("Heading: ");
    // Serial.print(heading);
    // Serial.print("°  Servo angle: ");
    // Serial.println(targetAngle);

    delay(500); // Stability delay

    // Read accelerometer and gyroscope data
    sensors_event_t accelEvent, gyroEvent;
    float x, y, z;
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(x, y, z);
      Serial.print(x);
      Serial.print('\t');
      Serial.print(y);
      Serial.print('\t');
      Serial.println(z);
    }
    lis3mdl.getEvent(&magEvent);  // Read magnetometer

    // Retrieve Temperature and Pressure from ENS220
    float temperature = ens220.getTempCelsius();
    float pressure = ens220.getPressureHectoPascal();

    // ENS220 Continuous mode data reading
    ContinuousModeWithFIFO_loop();

    // Telemetry Transmission
    if (telemetryEnabled) {
        String telemetry = String(TEAM_ID) + "," + "Time: " + missionTime + "s," + packetCount + ",FLIGHT,ACTIVE," + 
                          gpsAltitude + "," + currentVoltage + "," + x + "," + y + "," + z + "," +
                          gyroEvent.gyro.x + "," + gyroEvent.gyro.y + "," + gyroEvent.gyro.z + "," +
                          magEvent.magnetic.x + "," + magEvent.magnetic.y + "," + magEvent.magnetic.z + "," +
                          latitude + "," + longitude + "," + satellites + "," +
                          "Temperature(C):" + temperature + "," + "Pressure(hPa):" + pressure;

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

    // Delay before next telemetry update
    delay(100);
}
