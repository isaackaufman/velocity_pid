// motor pid libraries
#include "Arduino.h"
#include "MotorVelocity.h"

int SPEED = 3.5;

#define REV_BASE (4741.44)  // revolution is 1920 counts
#define REV_FLAG (464.64)   // https://www.pololu.com/category/115/25d-mm-metal-gearmotors

/* motor hookups */
// left front
#define PWM1    (9)
#define DIR1    (48)
// right front
#define PWM2    (8)
#define DIR2    (46)
// left rear
#define PWM3    (10)
#define DIR3    (50)
// right rear
#define PWM4    (11)
#define DIR4    (52)

/* encoders */
// left front
#define ENC1_A  (42)
#define ENC1_B  (44)
// right front
#define ENC2_A  (24)
#define ENC2_B  (26)
// left rear
#define ENC3_A  (32)
#define ENC3_B  (34)
// right rear
#define ENC4_A  (36)
#define ENC4_B  (38)

/* PIDs */
// left front
#define P1 (6.0f)
#define I1 (0.00001f)
#define D1 (0.0f)
// right front
#define P2 (6.0f)
#define I2 (0.00001f)
#define D2 (0.0f)
// left rear
#define P3 (6.0f)
#define I3 (0.00001f)
#define D3 (0.0f)
// right rear
#define P4 (6.0f)
#define I4 (0.00001f)
#define D4 (0.0f)
// flag
#define P5 (2.0f)
#define I5 (0.00001f)
#define D5 (0.0f)
// treasure
#define P6 (0.005f)
#define I6 (0.0f)
#define D6 (0.0f)

/* control flow */
#define PID_PERIOD  (10000)    // rate at which PID updates in microseconds

uint32_t last_run = 0,      // last PID run in miliseconds
         curr_run = 0;

MotorVelocity<PWM1, DIR1, ENC1_A, ENC1_B> left_front(P1, I1, D1);
MotorVelocity<PWM2, DIR2, ENC2_A, ENC2_B> right_front(P2, I2, D2);
MotorVelocity<PWM3, DIR3, ENC3_A, ENC3_B> left_rear(P3, I3, D3);
MotorVelocity<PWM4, DIR4, ENC4_A, ENC4_B> right_rear(P4, I4, D4);


// serial stuff with Pi
String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete


void setup() {

    // setup serial
    Serial.begin(115200);
    inputString.reserve(200);

    // set directions
    left_rear.reverseMotor();
    left_front.reverseMotor();
    right_rear.reverseMotor();
    right_front.reverseMotor();

    // set targets
    left_front.setTarget(SPEED);
    right_front.setTarget(SPEED);
    left_rear.setTarget(SPEED);
    right_rear.setTarget(SPEED);
}

void loop() {
    if (inputString == "FORWARD") {
        driveForward();
    } else if (inputString == "BACKWARD") {
        driveBackward();
    } else if (inputString == "LEFT") {
        driveLeft();
    } else if (inputString == "RIGHT") {
        driveRight();
    } else if (inputString == "CW") {
        rotateCW();
    } else if (inputString == "CCW") {
        rotateCCW();
    } else if (inputString == "STOP") {
        stopRobot();
    } else if (inputString == "FLAG") {
        extendFlag();
    } else if (inputString == "CHEST") {
        grabChest();
    }
    // } else if (inputString == "NEWHEADING") {
    //     gyro1.reset();
    //     setPointHeading = gyro1.getZ();
    //     initPIDs();
    // }

    //reads the input from the pi
    if (stringComplete) {
        // clear the string:
        inputString = "";
        stringComplete = false;
    }

    // determine if it's time to run the PID calculations again
    if (micros() - last_run > PID_PERIOD) {
        int left_front_command = left_front.update();
        int right_front_command = right_front.update();
        int left_rear_command = left_rear.update();
        int right_rear_command = right_rear.update();

        left_front.setSpeed(left_front_command);
        right_front.setSpeed(right_front_command);
        left_rear.setSpeed(left_rear_command);
        right_rear.setSpeed(right_rear_command);

        last_run = micros();
    }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == 33) {
      inputString = "";
    } else if (inChar == '\n') {
      int dumm = inputString.indexOf("@");
      String speedString = inputString.substring(dumm + 1, inputString.length() - 1);
      inputString = inputString.substring(0, dumm);
      if (inputString != "IRCODE") {
        // leftVel = speedString.toInt();
        // rightVel = speedString.toInt();
      } else {
//        displayNum(speedString.toInt());
      }
      stringComplete = true;
    }
  }
  Serial.println(inputString);
}

void setTargets(float lf, float rf, float lr, float rr) { // 1 or -1 for direction, 0 for stop
    left_front.setTarget(lf * SPEED);
    left_front.setTarget(rf * SPEED);
    left_front.setTarget(lr * SPEED);
    left_front.setTarget(rr * SPEED);
}

void driveForward() {
    setTargets(1, 1, 1, 1);
}
void driveBackward() {
    setTargets(-1, -1, -1, -1);
}
void driveLeft() {
    setTargets(1, -1, 1, -1);
}
void driveRight() {
    setTargets(-1, 1, -1, 1);
}
void rotateCW() {
    setTargets(-1, -1, 1, 1);
}
void rotateCCW() {
    setTargets(1, 1, -1, -1);
}
void stopRobot() {
    setTargets(0, 0, 0, 0);
}
