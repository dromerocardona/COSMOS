#include <Arduino.h>
#include <ens220.h>
#include "i2c_interface.h"

using namespace ScioSense;

#define I2C_ADDRESS 0x20
#define BUZZER_PIN 3
#define GREEN_LED 5
#define BLUE_LED 4

ENS220 ens220;
I2cInterface i2c_1;

float startPressure = 0;
float lastAltitude = 0;
float apogeeAltitude = 0;
unsigned long lastTime = 0;
bool landed = false;
bool launchDetected = false;
bool descending = false;

float calculateAltitude(float pressure) {
    return (1.0f - pow(pressure / startPressure, 0.1903f)) * 44330.0f; 
}

void setup() {
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);

    digitalWrite(GREEN_LED, HIGH); // green means it's good to go
    digitalWrite(BLUE_LED, LOW);   // Blue turns on during descent

    Wire.begin();
    i2c_1.begin(Wire, I2C_ADDRESS);

    ens220.begin(&i2c_1)

    ens220.setDefaultConfiguration();
    ens220.setPressureConversionTime(ENS220::PressureConversionTime::T_16_4);
    ens220.setOversamplingOfPressure(ENS220::Oversampling::N_128);
    ens220.setOversamplingOfTemperature(ENS220::Oversampling::N_128);
    ens220.setPressureTemperatureRatio(ENS220::PressureTemperatureRatio::PT_1);
    ens220.setStandbyTime(ENS220::StandbyTime::OneShotOperation);
    ens220.setPressureDataPath(ENS220::PressureDataPath::Direct);
    ens220.writeConfiguration();

    // Get initial pressure for reference
    ens220.singleShotMeasure(ENS220::Sensor::TemperatureAndPressure);
    ens220.waitSingleShot();
    if (ens220.update() == ENS220::Result::Ok && hasFlag(ens220.getDataStatus(), ENS220::DataStatus::PressureReady)) {
        startPressure = ens220.getPressureHectoPascal();
    }
}

void loop() {
    ens220.singleShotMeasure(ENS220::Sensor::TemperatureAndPressure);
    ens220.waitSingleShot();
    
    if (ens220.update() == ENS220::Result::Ok) {
        if (hasFlag(ens220.getDataStatus(), ENS220::DataStatus::PressureReady)) {
            float pressure = ens220.getPressureHectoPascal();
            float altitude = calculateAltitude(pressure);

            // Detect launch if altitude increases by at least 10m
            if (!launchDetected && altitude > 10) {
                launchDetected = true;
            }

            // Track apogee (highest altitude)
            if (launchDetected && altitude > apogeeAltitude) {
                apogeeAltitude = altitude;
            }

            // Detect descent (altitude decreasing after apogee)
            if (launchDetected && !descending && altitude < (apogeeAltitude - 1)) { 
                descending = true;
                digitalWrite(GREEN_LED, LOW);  // Turn OFF green at apogee
                digitalWrite(BLUE_LED, HIGH);  // Turn ON blue for descent
            }

            // Detect landing (stable altitude for 3s)
            if (descending) {
                if (abs(altitude - lastAltitude) < 0.5) { 
                    if (millis() - lastTime > 3000) { 
                        landed = true;
                    }
                } else {
                    lastTime = millis(); 
                }
            }

            lastAltitude = altitude;
        }
    }

    if (landed) {
        digitalWrite(BUZZER_PIN, HIGH);
    }

    delay(100);
}
