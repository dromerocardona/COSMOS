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
//#include <TinyGPS.h>  REMOVE IN FUTURE
#include <Arduino.h>
#include "i2c_interface.h"
#include "FeedBackServo.h"
#include <Adafruit_NeoPixel.h>


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
// #define DATA_PIN    3
#define LED_DATA 5
#define NUM_LEDS 5 // Number of LEDs for FastLED


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
--------------------------------------------------------------------------------------------------------------------------------------
Also- since time is being attained from either the GCS or GPS only once, the FSW needs to increment the time. Here is a program that
could work to do this if implemented:

String currentTime = "00:00:00"; //replace with the time given by either GCS or GPS
*/

// Variables to track timing
unsigned long previousMillis = 0;
const long interval = 1000;
//updateTime(currentTime); calling this function.  input a string

// Sensor objects
ScioSense::ENS220 ens220;
//TinyGPS gps; // GPS sensor
Adafruit_LIS3MDL lis3mdl;// Magnetometer
// Set feedback signal pin number for the servo
// FeedBackServo servo = FeedBackServo(FEEDBACK_PIN);
float apogeeAltitude = 0.0;
unsigned long landedTime = 0;
unsigned long lastOrientationTime = 0;
float lastOrientationX = 0.0, lastOrientationY = 0.0, lastOrientationZ = 0.0;

// Variables
float voltageDividerFactor = 0.012089; // Adjust based on resistor values in voltage divider
unsigned int packetCount = 0;
File dataFile;
bool telemetryEnabled = false;  // Telemetry Control
float lastTransmissionTime = 0; // Last time of telemetry transmission
char currentTime[9] = "00:00:00"; // Mission time in "HH:MM:SS"
char lastCommand[32] = ""; // Last received command
char gpsTime[9] = "00:00:00"; // GPS time in "HH:MM:SS"

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
#define CAMERA_PIN 2  // Define CAMERA_PIN (adjust as needed)

float simulatedAltitude = 0.0;      // Altitude derived from simulated pressure

const int historySize = 10; // Fixed size of the history arrays
float altitudeHistory[historySize];
float velocityHistory[historySize];
unsigned long timestampHistory[historySize]; // To store time in milliseconds

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
        // Add calibration action here if needed
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
                    digitalWrite(CAMERA_PIN, HIGH); // Using CAMERA_PIN for simplicity
                    Serial.println("MEC CAMERA BLADE ON - Camera powered ON.");
                } else if (strcmp(field4, "OFF") == 0) {
                    digitalWrite(CAMERA_PIN, LOW);
                    Serial.println("MEC CAMERA BLADE OFF - Camera powered OFF.");
                }
            } else if (strcmp(field3, "GROUND") == 0 && num >= 4) {
                if (strcmp(field4, "ON") == 0) {
                    digitalWrite(CAMERA_PIN, HIGH); // Using CAMERA_PIN for simplicity
                    Serial.println("MEC CAMERA GROUND ON - Camera powered ON.");
                } else if (strcmp(field4, "OFF") == 0) {
                    digitalWrite(CAMERA_PIN, LOW);
                    Serial.println("MEC CAMERA GROUND OFF - Camera powered OFF.");
                }
            } else if (strcmp(field3, "STABLE") == 0 && num >= 4) {
              Serial.println("MEC CAMERA STABLE command received.");
              // Add camera stabilization action here
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
//FIX!!------------------------------------------------------------------------------------------------------------
while (ens220.begin(&i2c_1) != true)
{
  Serial.println("Waiting for I2C to start");
  delay(1000);
}
//FIX!!------------------------------------------------------------------------------------------------------------
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

void setup(){
  Serial.begin(115200);         // Debugging output
  Serial1.begin(9600);          // XBee communication
  Wire.begin();

  // Initialize camera control pin (e.g., for power on/off)
  pinMode(CAMERA_PIN, OUTPUT);  // Set camera control pin to output
  digitalWrite(CAMERA_PIN, LOW);  // Make sure camera is OFF initially

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    pixels.setPixelColor(150, 0, 0);
  }

  // Initialize RPM sensor
  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmISR, RISING);

  // Initialize GPS (using Serial1 to communicate with GPS)
  Serial1.begin(9600);

  // Initialize LSM6DSO32 (Accelerometer and Gyroscope)
  if (!IMU.begin()) {  // Corrected reference to the lsm6dso32 object
    Serial.println("Error initializing LSM6DSOX!");
    ;
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
  }
  else{
    Serial.println("LIS3MDL Found!");
  }
  

  // Set up LIS3MDL settings
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  //servo.setServoControl(SERVO_PIN);
  //servo.setKp(1.0);  // You can adjust the PID controller gain

  // Initialize ENS220 sensor
#ifdef DEBUG_ENS220
  ens220.enableDebugging(Serial);
#endif

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
  float latitude = 0.0, longitude = 0.0, gpsAltitude = 0.0;

  /*
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
  if (gps._time.isValid()) {
    snprintf(gpsTime, sizeof(gpsTime), "%02d:%02d:%02d", gps._time.hour(), gps._time.minute(), gps._time.second());
  }
*/
  // Read magnetometer data
  sensors_event_t magEvent;
  lis3mdl.getEvent(&magEvent);

  
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

  // ENS220 Continuous mode data reading
  ContinuousModeWithFIFO_loop();

  /*------------------CORE OPERATIONS----Sensor Data----altitude and simulation mode-----------*/
  // Use simulated pressure if in simulation mode
  if (simulationMode) {
    simulatedAltitude = (1 - pow(simulatedPressure / 1013.25, 0.190284)) * 145366.45;  // Approximation formula
    gpsAltitude = simulatedAltitude;
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
               "%s,%s,%u,%s,%s,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%s,%.1f,%.4f,%.4f,%u,%s,COSMOS",
               TEAM_ID, currentTime, packetCount, mode, state, gpsAltitude, temperature, pressure, currentVoltage,
               gyroX, gyroY, gyroZ, accelX, accelY, accelZ,
               magEvent.magnetic.x, magEvent.magnetic.y, magEvent.magnetic.z, rpm, gpsTime,
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
  float heading = atan2(magEvent.magnetic.y, magEvent.magnetic.x);
  heading = heading * 180 / PI; // Convert to degrees
  if (heading < 0) heading += 360; // Ensure 0-360 range

  //used for some state trasitions
  updateAltitudeHistory(altitudeHistory, timestampHistory, gpsAltitude, historySize); //GPS ALTITUDE IS NOT SUFICIENT FOR THIS FUNCTION!!! REMOVE AS SOON AS POSSIBLE!!!
  updateVelocityHistory(altitudeHistory, velocityHistory, timestampHistory, historySize);
  /*---------------------------------STATE DEPENDENT OPERATIONS---------------------------------*/

  //it is very important to fill this with the correct logic for trasitions and contents for each state ex. PID control in separated and deployed
  switch (flightState) {//switch statement so we have smooth-looking code :)
  ////////////////////////////////////////////////////////////////////////
    case LAUNCH_PAD:
      // Code for launch state
      updateFlightState(gpsAltitude, velocityHistory[0], accelX, accelY, accelZ);
      break;
  ////////////////////////////////////////////////////////////////////////
    case ASCENT:
      // Code for ascent state
      updateFlightState(gpsAltitude, velocityHistory[0], accelX, accelY, accelZ);
      break;
  ////////////////////////////////////////////////////////////////////////
    case APOGEE:
      // Code for ascent state
      updateFlightState(gpsAltitude, velocityHistory[0], accelX, accelY, accelZ);
    break;
  ////////////////////////////////////////////////////////////////////////
    case DESCENT:
      // Code for separated state
      updateFlightState(gpsAltitude, velocityHistory[0], accelX, accelY, accelZ);
      break;
  ////////////////////////////////////////////////////////////////////////
    case LANDED:
      /*---------------------------------STATE DEPENDENT OPERATIONS----PID--------------------------*/
      // get input from the system
      input = heading;

      // Calculate the error
      error = setpoint - input; // Calculate the error based on difference between setpoint and input
      if (error > 180) {
        error = error - 360;// account for the clossest path to the setpoint being across the 0° line
      }
      else if (error < -180) {
        error = error + 360;// account for the clossest path to the setpoint being across the 0° line the other way
      }
      //this method of error calculation may need to be changed based on the system

      // Calculate the integral term
      integral += error;

      // Calculate the derivative term
      float derivative = error - lastError;

      // Calculate the PID output
      output = K_proportional * error + K_integral * integral + K_derivative * derivative;

      // Update the system based on the PID output
      // updateSystem(output);

      // Store the current error as the last error to prepare for the next iteration
      lastError = error;

      // Print the current values for debugging
      //Serial.print("Input: ");
      //Serial.print(input);
      //Serial.print(", Output: ");
      //Serial.println(output);

      //END OF PID CONTROL

      // Serial.print("Heading: ");
      // Serial.print(heading);
      // Serial.print("°  Servo angle: ");
      // Serial.println(targetAngle);
      updateFlightState(gpsAltitude, velocityHistory[0], accelX, accelY, accelZ);
      if (avg(velocityHistory, historySize) < 1) {
        flightState = LANDED;
      }
      break;
  ////////////////////////////////////////////////////////////////////////
  }
}
/*-------------------------------------------REPEAT-------------------------------------------*/
