#pragma once
#include "PID.h"

const int8_t encoder_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

template<int pwm_pin, int dir_pin_1, int dir_pin_2, int enca_pin, int encb_pin>
class Motor {
    protected:
        int direction_coef = 1, velocity_control = 0;

        uint32_t last_micros = 0;
        int32_t last_encoder = 0;

        PID * pid;
        float target = 0.0;

        static volatile int32_t encoder_count;
        static uint8_t enc_val;

    public:
        // constructor
        Motor(float p, float i, float d);            // pins
        
        // motor speed commands
        void setSpeed(int new_speed);           // 0-255
        void stop(void);
        virtual int update(void) = 0;

        // motor direction commands
        void reverseMotor(void);   // 1 for forward is CCW (default), 0 for forward is CW
        void setDirection(int new_direction);
        
        // command the PID
        void setTarget(float new_target);
        void updateTarget(float new_target);

        // encoder isr
        static void encoder_isr(void);

};

template<int pwm_pin, int dir_pin_1, int dir_pin_2, int enca_pin, int encb_pin>
volatile int32_t Motor<pwm_pin, dir_pin_1, dir_pin_2, enca_pin, encb_pin>::encoder_count = 0;

template<int pwm_pin, int dir_pin_1, int dir_pin_2, int enca_pin, int encb_pin>
uint8_t Motor<pwm_pin, dir_pin_1, dir_pin_2, enca_pin, encb_pin>::enc_val = 0;

template<int pwm_pin, int dir_pin_1, int dir_pin_2, int enca_pin, int encb_pin>
Motor<pwm_pin, dir_pin_1, dir_pin_2, enca_pin, encb_pin>::Motor(float p, float i, float d) {
//    attachInterrupt(digitalPinToInterrupt(enca_pin), encoder_isr, CHANGE);
//    attachInterrupt(digitalPinToInterrupt(encb_pin), encoder_isr, CHANGE);
    attachInterrupt(enca_pin, encoder_isr, CHANGE);
    attachInterrupt(encb_pin, encoder_isr, CHANGE);

    pinMode(dir_pin_1, OUTPUT);
    pinMode(dir_pin_2, OUTPUT);

    pid = new PID(p, i, d, 255);
    pid->reset();

}
template<int pwm_pin, int dir_pin_1, int dir_pin_2, int enca_pin, int encb_pin>
void Motor<pwm_pin, dir_pin_1, dir_pin_2, enca_pin, encb_pin>::setSpeed(int new_speed) {
    analogWrite(pwm_pin, new_speed);
}
template<int pwm_pin, int dir_pin_1, int dir_pin_2, int enca_pin, int encb_pin>
void Motor<pwm_pin, dir_pin_1, dir_pin_2, enca_pin, encb_pin>::stop() {
    analogWrite(pwm_pin, 0);
}
template<int pwm_pin, int dir_pin_1, int dir_pin_2, int enca_pin, int encb_pin>
void Motor<pwm_pin, dir_pin_1, dir_pin_2, enca_pin, encb_pin>::reverseMotor() {
    direction_coef = 0;
}
template<int pwm_pin, int dir_pin_1, int dir_pin_2, int enca_pin, int encb_pin>
void Motor<pwm_pin, dir_pin_1, dir_pin_2, enca_pin, encb_pin>::setDirection(int new_direction) {
    // forward (new_direction = 1): in1 = 0, in2 = 1
    // reverse (new_direction = 0): in1 = 1, in2 = 0
    digitalWrite(dir_pin_1, !new_direction);
    digitalWrite(dir_pin_2, new_direction);
}
template<int pwm_pin, int dir_pin_1, int dir_pin_2, int enca_pin, int encb_pin>
void Motor<pwm_pin, dir_pin_1, dir_pin_2, enca_pin, encb_pin>::setTarget(float new_target) {
    target = (direction_coef) ? new_target : -new_target;
    pid->reset();
}
template<int pwm_pin, int dir_pin_1, int dir_pin_2, int enca_pin, int encb_pin>
void Motor<pwm_pin, dir_pin_1, dir_pin_2, enca_pin, encb_pin>::updateTarget(float new_target) {
    target = (direction_coef) ? new_target : -new_target;
}
template<int pwm_pin, int dir_pin_1, int dir_pin_2, int enca_pin, int encb_pin>
void Motor<pwm_pin, dir_pin_1, dir_pin_2, enca_pin, encb_pin>::encoder_isr() {
    enc_val = (enc_val << 2) | (digitalRead(enca_pin) << 1) | digitalRead(encb_pin);

    encoder_count += encoder_table[enc_val & 0b1111];
}
