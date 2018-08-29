#pragma once
#include "Motor.h"

template<int pwm_pin, int dir_pin, int enca_pin, int encb_pin>
class MotorPosition : public Motor<pwm_pin, dir_pin, enca_pin, encb_pin> {
    public:
        MotorPosition(float p, float i, float d) : Motor<pwm_pin, dir_pin, enca_pin, encb_pin>(p,  i,  d) {};
        int update(void);
};

template<int pwm_pin, int dir_pin, int enca_pin, int encb_pin>
int MotorPosition<pwm_pin, dir_pin, enca_pin, encb_pin>::update() {

    noInterrupts();
    float error = this->target - this->encoder_count;
    interrupts();

    float output = this->pid->update(error);

    if (output > 0) {
        this->setDirection(1);
    } else {
        this->setDirection(0);
    }

    Serial.println(this->encoder_count);

    return constrain(abs(output), 0, 255);

    // setSpeed(output);
}
