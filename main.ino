#include <Wire.h>
#include <Servo.h>
//#include "common_defs/defs_enum.h"

#define  MAIN_M0 4
#define  MAIN_M1 5
#define  MAIN_M2 2
#define  MAIN_M3 3
#define  MAIN_M4 6
#define  MAIN_M5 7
#define  MAIN_M6 8
#define  MAIN_M7 9
#define  MAIN_VID0_0 22 // bit 0 of video feed 0 selection
#define  MAIN_VID0_1 23
#define  MAIN_VID0_2 24
#define  MAIN_VID1_0 25 // bit 0 of video feed 1 selection
#define  MAIN_VID1_1 26
#define  MAIN_VID1_2 27

Servo M0;
Servo M1;
Servo M2;
Servo M3;
Servo M4;
Servo M5;
Servo M6;
Servo M7;

Servo servos[8] = { M0, M1, M2, M3, M4, M5, M6, M7 };
// int servo_pins[8] = { PINS.MAIN_M0,
//                       PINS.MAIN_M1,
//                       PINS.MAIN_M2,
//                       PINS.MAIN_M3,
//                       PINS.MAIN_M4,
//                       PINS.MAIN_M5,
//                       PINS.MAIN_M6,
                      // PINS.MAIN_M7 };
int servo_pins[8] = { MAIN_M0,
                      MAIN_M1,
                      MAIN_M2,
                      MAIN_M3,
                      MAIN_M4,
                      MAIN_M5,
                      MAIN_M6,
                      MAIN_M7 };


int POWER_BOARD = 0;
int SHUTOFF = 0;
int SHUTOFF_REQ = 'X';

String signal;
bool kill = false;

void setup() {
  Wire.begin();
  Serial2.begin(115200);
  Serial2.setTimeout(100);
  for (int s = 0; s < 8; s++) {
    servos[s].attach(servo_pins[s]); // attach servos to their pins
  }
}

void loop() {
  int motor; // which motor is being set
  int velocity; // speed motor is set to
  if (kill) {
    Serial2.println("KILL");
    Wire.beginTransmission(POWER_BOARD); 
    Wire.write(SHUTOFF); // TODO
    Wire.endTransmission();
    delay(50);
  } else {
    if (Serial2.available()) {
      signal = Serial2.readString();
      if (signal[0] == SHUTOFF_REQ) { // TODO
          kill = true;
      } else if (signal.length() == 7 && signal[1] == ':') {
        motor = signal[0] - '0'; // subtract value for char 0
        velocity = signal.substring(2,5).toInt();
        servos[motor].write(velocity);
        delay(15);
        Serial2.println(motor);
        Serial2.println(velocity);
      }
    }
  }
}
