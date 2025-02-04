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

// PINS AND DEFINITIONS
#define BATTERY_PIN A0 // Analog pin for voltage divider circuit
#define RPM_PIN 2      // Pin for Hall effect sensor
#define SD_CS_PIN 4    // Chip select pin for SD card
#define SERVO_PIN 9    // Servo pin for camera stabilization
#define I2C_ADDRESS 0x20 // I2C Address for ENS 220
#define SERIAL_BAUDRATE 57600 // Speed of Serial Communication with the computer (ENS220)
#define INTN_1 2 // Interrupt pin for ENS220
#define CAMERA_PIN 7 // RunCam

// Team ID
#define TEAM_ID "3195"

// Sensor objects
using namespace ScioSense; // ENS220
ENS220 ens220; // sensor object ENS220
TinyGPSPlus gps; // GPS sensor
Adafruit_LIS3MDL lis3mdl;// Magnetometer

// Variables
float voltageDividerFactor = 5.0; // Adjust based on resistor values in voltage divider
volatile unsigned long rpmCount = 0;
unsigned long lastRpmTime = 0;
unsigned int packetCount = 0;
float cameraposition = 0;
File dataFile;
bool telemetryEnabled = false;  // Telemetry Control

// Servo control (PWM)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 150   // Minimum pulse length count (out of 4096)
#define SERVOMAX 600   // Maximum pulse length count (out of 4096)
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates
uint8_t servonum = 0; // Servo channel number

// Simulation mode variables
bool simulationMode = false;
float simulatedPressure = 1013.25;  // Default sea level pressure in hPa
float simulatedAltitude = 0.0;      // Altitude derived from simulated pressure

// Interrupt Service Routine for RPM counting
void rpmISR() {
rpmCount++;
}

// Function to set the servo pulse
void setServoPulse(uint8_t n, double pulse) {
double pulselength = 1000000.0; // 1,000,000 us per second
pulselength /= SERVO_FREQ;      // Analog servos run at ~50 Hz updates
pulselength /= 4096.0;          // 12 bits of resolution
pulse *= 1000000.0;             // Convert input seconds to microseconds
pulse /= pulselength;
pwm.setPWM(n, 0, pulse);
}

// Function to make the camera always face north
void pointCameraNorth(float heading) {
// The target position is 0 degrees (facing north)
// We can calculate the angle difference between the current heading and the target
float angleDifference = 0 - heading;

// Map the difference to the servo range (0 to 180 degrees)
int pulseLength = map(angleDifference, -90, 90, SERVOMIN, SERVOMAX);

// Ensure the pulse length is within the servo's valid range
pulseLength = constrain(pulseLength, SERVOMIN, SERVOMAX);

// Set the PWM for the servo to position the camera
pwm.setPWM(servonum, 0, pulseLength);

delay(500);  // Wait for the servo to adjust
}

void handleCommand(String command) {
command.trim();  // Remove any leading or trailing spaces

if (command == "SIM_ENABLE") {
simulationMode = true;
Serial.println("Simulation mode enabled.");
} 
else if (command == "SIM_ACTIVATE") {
if (simulationMode) {
Serial.println("Simulation activated. Using simulated pressure values.");
} else {
Serial.println("Simulation not enabled yet.");
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

void setup() {
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

// Initialize PWM for servo control
pwm.begin();
pwm.setOscillatorFrequency(27000000);  // Set oscillator frequency to 27MHz
pwm.setPWMFreq(SERVO_FREQ);  // Set PWM frequency for servos
delay(10);

// Initialize ENS220 sensor
Serial.begin(SERIAL_BAUDRATE);
Wire.begin();   
#ifdef DEBUG_ENS220
ens220.enableDebugging(Serial);
#endif
ContinuousModeWithFIFO_setup();

// Initialize variables
lastRpmTime = millis();
}

void loop() {

// Read data from XBee or Serial1
if (Serial1.available()) {
String command = Serial1.readStringUntil('\n');  // Read command from XBee
handleCommand(command);  // Process the command
}

unsigned long missionTime = millis() / 1000; // Mission time in seconds

// Read battery voltage
float currentVoltage = analogRead(BATTERY_PIN) * (5.0 / 1023.0) * voltageDividerFactor;

// Calculate RPM
unsigned long currentTime = millis();
float rpm = (rpmCount / (float)(currentTime - lastRpmTime)) * 60000.0;
rpmCount = 0;
lastRpmTime = currentTime;

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
// Use the simulated pressure value to calculate the altitude
simulatedAltitude = (1 - pow(simulatedPressure / 1013.25, 0.190284)) * 145366.45;  // Approximation formula
gpsAltitude = simulatedAltitude;
}

// Read Accelerometer, Gyroscope, and Magnetometer Data
sensors_event_t accelEvent, gyroEvent, magEvent;
float x, y, z;
if (IMU.accelerationAvailable()) {
IMU.readAcceleration(x, y, z);

Serial.print(x);
Serial.print('\t');
Serial.print(y);
Serial.print('\t');
Serial.println(z);
}
lis3mdl.getEvent(&magEvent);  // Magnetometer data

ContinuousModeWithFIFO_loop();// ENs 220

// Retrieve Temperature and Pressure from ENS220
float temperature = ens220.getTempCelsius();
float pressure = ens220.getPressureHectoPascal();

// Only send telemetry if telemetryEnabled is true
if (telemetryEnabled) {

// Prepare telemetry data
 String telemetry = String(TEAM_ID) + "," + "Time: " + missionTime + "s," + "," + packetCount + ",FLIGHT,ACTIVE," + gpsAltitude + "," +
          currentVoltage + "," + accelEvent.acceleration.x + "," + accelEvent.acceleration.y + "," + accelEvent.acceleration.z + "," +
          gyroEvent.gyro.x + "," + gyroEvent.gyro.y + "," + gyroEvent.gyro.z + "," +
          magEvent.magnetic.x + "," + magEvent.magnetic.y + "," + magEvent.magnetic.z + "," +
          latitude + "," + longitude + "," + satellites + "," +
          "Temperature(C):" + temperature + "," + "Pressure(hPa):" + pressure;

// Transmit telemetry via XBee
Serial1.println(telemetry);

// Save telemetry to SD card
dataFile = SD.open("telemetry.txt", FILE_WRITE);
if (dataFile) {
dataFile.println(telemetry);
dataFile.close();
} else {
Serial.println("Error writing to SD card!");
}

// Increment packet count
packetCount++;
}

}
