//Code for calibrating the 2025 CanSat Team Cosmos release mechanism servo
//Written by Barrett Twining

#include <Servo.h>

Servo myservo;  // create Servo object to control a servo

int userInput = 1500;

void setup() {
  myservo.attach(13);
  Serial.begin(9600);
    // attaches the servo on pin 9 to the Servo object
}

void loop() {

while (Serial.available() == 0) {
    }
    
Serial.println("Input Desired Pulse Width (in Microseconds)");
userInput = Serial.parseInt();
myservo.writeMicroseconds(userInput);
Serial.print("Pulse Width Set to: ");
Serial.println(userInput);
delay(50);
}
