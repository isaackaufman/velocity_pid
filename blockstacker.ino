#include "Arduino.h"

#define REV_WHEEL    (3591.84) // https://www.pololu.com/product/4866

// Motor Interfaces
#define PWM_LEFT     (0)
#define DIR1_LEFT    (0)
#define DIR2_LEFT    (0)
#define ENCA_LEFT    (0)
#define ENCB_LEFT    (0)

#define PWM_RIGHT    (0)
#define DIR1_RIGHT   (0)
#define DIR2_RIGHT   (0)
#define ENCA_RIGHT   (0)
#define ENCB_RIGHT   (0)

// PID Controllers
#define KP_LEFT      (6.0f)
#define KI_LEFT      (0.00001f)
#define KD_LEFT      (0.0f)

#define KP_RIGHT     (6.0f)
#define KI_RIGHT     (0.00001f)
#define KD_RIGHT     (0.0f)

#define PID_PERIOD_MS  (10) // PID update rate in milliseconds
uint32_t last_run_ms = 0; // last PID run in milliseconds

MotorVelocity<PWM_LEFT, DIR1_LEFT, DIR2_LEFT, ENCA_LEFT, ENCB_LEFT> left(KP_LEFT, KI_LEFT, KD_LEFT);
MotorVelocity<PWM_RIGHT, DIR1_RIGHT, DIR2_RIGHT, ENCA_RIGHT, ENCB_RIGHT> right(KP_RIGHT, KI_RIGHT, KD_RIGHT);

// Serial
String input = ""; // Input command over serial
const char startMarker = '<';
const char endMarker = '>';

// Dead Reckoning
double dx = 0.0f;
double dy = 0.0f;
double dtheta = 0.0f;

void setup() {
    Serial.begin(115200);
    input.reserve(200);

    left.setTarget(0);
    right.setTarget(0);
    last_run_ms = millis();
}

void loop() {
    if (millis() - last_run_ms > PID_PERIOD_MS) {
        left.setSpeed(left.update());
        right.setSpeed(right.update());

        last_run_ms = millis()
    }
}

void serialEvent() {
    while (Serial.available() > 0) {
        char input_char = (char) Serial.read();

        if (input_char == endMarker) {
            parseInput();
            replyOutput();
        } else if (input_char == startMarker) {
            input = ""
        } else {
            input += input_char;
        }
    }
}

void parseInput() {
    char* left_omega = strtok(input, ",");
    int left_motor_speed = atof(left_omega)

    char* right_omega = strtok(NULL, ",");
    int right_motor_speed = atof(right_omega);
}

void replyOutput() {
    Serial.print("<");
    Serial.print(dx);
    Serial.print(",");
    Serial.print(dy);
    Serial.print(",");
    Serial.print(dtheta);
    Serial.println(">");

    // Reset dead reckoning
    dx = 0.0f;
    dy = 0.0f;
    dtheta = 0.0f;
}