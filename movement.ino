#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"
#include "Endo-Continuum-Robot.h"

#ifndef Continuum_h
#define Continuum_h
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"


class Continuum
{
private:
    // Define Variables
    int _SERVOMIN = 120;    // This is the 'minimum' pulse length count (out of 4096)
    int _SERVOMAX = 500;    // This is the 'maximum' pulse length count (out of 4096)
    int _USMIN = 600;       // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
    int _USMAX = 2400;      // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
    int _SERVO_FREQ = 50;   // Analog servos run at ~50 Hz updates
    double _angleArray[16]; //
    int _sections = 3;              // Number of continuum robot sections
    Adafruit_PWMServoDriver pwm;    // Adafruit PWM Servo driver definition
    double _cardTuner = 0.5;        //
    double _sectionTuner = 0;       //

public:
    // Define methods
    Continuum(int startAngle);                                            //
    Continuum(int startAngle, int sections);                              //
    void pwmSetup();                                                      //
    void setCardinalTuner(double cardTuner);                              //
    void setSectionTuner(double sectionTuner);                            //
    void setPWMBounds(int SERVOMIN, int SERVOMAX);                        //
    double pulseLength(double angle);                                     //
    void startPull();                                                     //
    void pullIncrement(int section, double incrementAngle, int card);     //
    void setAngles(double angleArray[]);                                  //
    void singleServoAdjust(int section, double angle, int card);          //
    double currentAngle(int section, int card);                           // 
    void easyPull(int section, int incrementAngle, int card);             //
};
#endif


Continuum::Continuum(int startAngle)
{
    for (int i = 0; i < 12; i++)
    {
        _angleArray[i] = startAngle;
    }
    pwm = Adafruit_PWMServoDriver();
}
Continuum::Continuum(int startAngle, int sections)
{
    _sections = sections;
    for (int i = 0; i < sections * 4; i++)
    {
        _angleArray[i] = startAngle;
    }
    pwm = Adafruit_PWMServoDriver();
}
void Continuum::pwmSetup()
{
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(_SERVO_FREQ);
}
void Continuum::setPWMBounds(int SERVOMIN, int SERVOMAX)
{
    _SERVOMIN = SERVOMIN;
    _SERVOMAX = SERVOMAX;
}
void Continuum::setCardinalTuner(double cardTuner)
{
    _cardTuner = cardTuner;
    Serial.print("Directional Tuning: ");
    Serial.println(_cardTuner);
}
void Continuum::setSectionTuner(double sectionTuner)
{
    _sectionTuner = sectionTuner;
    Serial.print("Sectional Tuning: ");
    Serial.println(_sectionTuner);
}
double Continuum::pulseLength(double angle) { return constrain(map(angle, 0, 180, _SERVOMIN, _SERVOMAX), _SERVOMIN, _SERVOMAX); }
void Continuum::startPull()
{
    Serial.println("{");
    for (int i = 0; i < _sections * 4; i++)
    {
        Serial.print(_angleArray[i]);
        Serial.print("\t");
        pwm.setPWM(i, 0, pulseLength(_angleArray[i]));
    }
    Serial.println("}");
}
void Continuum::pullIncrement(int section, double incrementAngle, int card)
{
    for (int i = 0; i < section - 1; i++)
    {
        _angleArray[4 * i + card] += incrementAngle;
        _angleArray[4 * i + ((card + 3) % 4)] += _cardTuner * (incrementAngle * _sectionTuner);
        _angleArray[4 * i + ((card + 1) % 4)] += _cardTuner * (incrementAngle * _sectionTuner);
    }
    for (int i = section - 1; i < _sections; i++)
    {
        _angleArray[4 * i + card] += (incrementAngle * section) / (i + 1);
        _angleArray[4 * i + ((card + 3) % 4)] += _cardTuner * ((incrementAngle * section) / (i + 1));
        _angleArray[4 * i + ((card + 1) % 4)] += _cardTuner * ((incrementAngle * section) / (i + 1));
    }
}
void Continuum::setAngles(double angleArray[])
{
    for (int i = 0; i < _sections * 4; i++)
    {
        _angleArray[i] = angleArray[i];
    }
    startPull();
}
void Continuum::singleServoAdjust(int section, double angle, int card)
{
    _angleArray[4 * (section - 1) + card] = angle;
    startPull();
}
double Continuum::currentAngle(int section, int card)
{
    return _angleArray[4 * (section - 1) + card];
}

void Continuum::easyPull(int section, int incrementAngle, int card)
{
    float angle = ((card % 90) * M_PI) / 180;
    if (card % 90 == 0)
    {
        pullIncrement(section, incrementAngle, card / 90);
    }
    else
    {
        _angleArray[4 * (section - 1) + (card / 90)] += incrementAngle * (tan(angle) + 2 * (180 / incrementAngle) - 2) / 1 + tan(angle);
        _angleArray[4 * (section - 1) + ((card / 90 + 1) % 4)] += incrementAngle * (2 * (180 / incrementAngle) - 1 - (tan(angle) + 2 * (180 / incrementAngle) - 2) / 1 + tan(angle));
    }
    startPull();
}



