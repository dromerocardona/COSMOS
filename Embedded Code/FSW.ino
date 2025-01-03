#include <Adafruit_PWMServoDriver.h>
#include <ScioSense_ENS220.h>
#include <ens220.h>
#include <utils.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>

// Pins
#define BATTERY_PIN A0 // Analog pin for voltage divider circuit
#define RPM_PIN 2      // Pin for Hall effect sensor
#define SD_CS_PIN 4    // Chip select pin for SD card
#define SERVO_PIN 9    // Servo pin for camera stabilization

// Team ID
#define TEAM_ID "3195"

// Sensor objects
Adafruit_BNO055 bno = Adafruit_BNO055(55);
TinyGPSPlus gps;

// Variables
float voltageDividerFactor = 5.0; // Adjust based on resistor values in voltage divider
volatile unsigned long rpmCount = 0;
unsigned long lastRpmTime = 0;
unsigned int packetCount = 0;
float northAngle = 0.0;
float cameraposition = 90;
File dataFile;

// Servo control (PWM)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 150   // Minimum pulse length count (out of 4096)
#define SERVOMAX 600   // Maximum pulse length count (out of 4096)
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates
uint8_t servonum = 0; // Servo channel number


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
  // The target position is 90 degrees (facing north)
  // We calculate the angle difference between the current heading and the target
  float angleDifference = 90.0 - heading;

  // Map the difference to the servo range (0 to 180 degrees)
  int pulseLength = map(angleDifference, -90, 90, SERVOMIN, SERVOMAX);
  
  // Ensure the pulse length is within the servo's valid range
  pulseLength = constrain(pulseLength, SERVOMIN, SERVOMAX);

  // Set the PWM for the servo to position the camera
  pwm.setPWM(servonum, 0, pulseLength);

  delay(500);  // Wait for the servo to adjust
}





void setup() {
  Serial.begin(9600);          // Debugging output
  Serial1.begin(9600);         // XBee communication
  Wire.begin();

  // Initialize BNO055
  if (!bno.begin()) {
    Serial.println("BNO055 initialization failed!");
    while (1);
  }
  bno.setExtCrystalUse(true);

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

// Initialize PWM for servo control
pwm.begin();
pwm.setOscillatorFrequency(27000000);  // Set oscillator frequency to 27MHz
pwm.setPWMFreq(SERVO_FREQ);  // Set PWM frequency for servos
delay(10);

  // Initialize variables
  lastRpmTime = millis();
}

void loop() {
  unsigned long missionTime = millis() / 1000; // Mission time in seconds

  // Read battery voltage
  float currentVoltage = analogRead(BATTERY_PIN) * (5.0 / 1023.0) * voltageDividerFactor;
 

  // Read BNO055 orientation and acceleration
  sensors_event_t orientationEvent, accelEvent, magEvent;
  bno.getEvent(&orientationEvent, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  float heading = orientationEvent.orientation.x;
  float roll = orientationEvent.orientation.z;
  float pitch = orientationEvent.orientation.y;

  // Stabilize camera to point north
  pointCameraNorth(heading);


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

  // Read temperature and pressure (placeholder for ENS220 sensor)
  float temperature = 25.0; // Placeholder value
  float pressure = 1013.25; // Placeholder value
  float altitude = 100.0;   // Placeholder value

  // Prepare telemetry data
  String telemetry = String(TEAM_ID) + "," + missionTime + "," + packetCount + ",FLIGHT,ACTIVE," + altitude + "," +
                     temperature + "," + pressure + "," + currentVoltage + "," + roll + "," + pitch + "," + heading + "," +
                     accelEvent.acceleration.x + "," + accelEvent.acceleration.y + "," + accelEvent.acceleration.z + "," +
                     magEvent.magnetic.x + "," + magEvent.magnetic.y + "," + magEvent.magnetic.z + ",0," +
                     gps.time.value() + "," + gpsAltitude + "," + latitude + "," + longitude + "," + satellites + ",NONE";

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

  // Delay for telemetry interval
  delay(1000);
}
