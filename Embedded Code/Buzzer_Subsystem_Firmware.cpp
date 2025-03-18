#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <ScioSense_ENS220.h>
#include <Wire.h>

const int buzzerPin = 0; // Pin for the buzzer
using namespace ScioSense;
ENS220 sensor;
float previousAltitude = 0.0; // Store the previous altitude reading
float descentThreshold = 1.0; // Threshold for detecting descent (in meters)

void setup() {
  Serial.begin(9600); // Start serial communication
  pinMode(buzzerPin, OUTPUT); // Set buzzer pin as output
  sensor.begin();
}

void loop() {
  float altitude = sensor.readAltitude(); // Read the current altitude
  Serial.println(altitude);

  // Check if altitude is below the threshold and is descending
  if (altitude < previousAltitude - descentThreshold) {
    digitalWrite(buzzerPin, HIGH); // Turn buzzer ON
  }
  
  previousAltitude = altitude; // Update previous altitude for the next comparison
  delay(1000); // Wait for 1 second before the next reading
}
