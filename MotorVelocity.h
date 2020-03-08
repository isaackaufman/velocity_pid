#pragma once
#include "Motor.h"

template<int pwm_pin, int dir_pin_1, int dir_pin_2, int enca_pin, int encb_pin>
class MotorVelocity : public Motor<pwm_pin, dir_pin_1, dir_pin_2, enca_pin, encb_pin> {
    public:
        MotorVelocity(float p, float i, float d) : Motor<pwm_pin, dir_pin_1, dir_pin_2, enca_pin, encb_pin>(p,  i,  d) {};
        int update(void);
};

template<int pwm_pin, int dir_pin_1, int dir_pin_2, int enca_pin, int encb_pin>
int MotorVelocity<pwm_pin, dir_pin_1, dir_pin_2, enca_pin, encb_pin>::update() {

    // velocity control
    noInterrupts();
    float dx = this->last_encoder - this->encoder_count;
    this->last_encoder = this->encoder_count;
    interrupts();
    uint32_t now = millis();
    float velocity = dx / (now - this->last_micros);
    this->last_micros = now;

    float error = this->target - velocity;
    float output = this->pid->update(error);

//    Serial.print("left rear error: "); Serial.println(error);

    if (output > 0) {
        this->setDirection(1);
    } else {
        this->setDirection(0);
    }

//    Serial.println(this->encoder_count);

    return constrain(abs(output), 0, 255);
}
