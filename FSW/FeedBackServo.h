#ifndef FeedbackServo_h
#define FeedbackServo_h

#include "Arduino.h"
#include "Servo.h"

// SET_THRESHOLD is used when an input pin (e.g. manual potmeter) is used.
#define SET_THRESHOLD 2

#define FEEDBACK_THRESHOLD 2

class FeedbackServo {
public:
    FeedbackServo();
    FeedbackServo(int pin, int feedbackPin);
    void begin(int pin, int feedbackPin);
    void setInputPin(int pin);
    void unsetInputPin();
    void loop();
    int setting();
    int settingMicro();
    void set(int setting);
    void setMicro(int setting);
    void adjust(int adjustment);
    void setReversed(bool reversed);
    int adjustedSetting();
    int adjustedSettingMicro();
    int adjustedSettingMicroReverted();
    bool inMicro();
    int feedback();
    void setMaxDelay(unsigned long delay);
    bool attached();
    void mconstrain(int min, int max);
private:
    Servo _servo;

    int _pin;
    int _feedbackPin;
    int _inputPin;

    int _setting;
    int _settingMicro;
    int _adjustment;
    unsigned long _setTime;

    int _lastPos;
    int _feedback;

    bool _reversed;

    unsigned long _maxDelay;

    int revert(int setting);

    int _min;
    int _max;
};

#endif
