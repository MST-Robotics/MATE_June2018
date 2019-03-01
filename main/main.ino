#include <Wire.h>
#include <Servo.h>
#include "defs_enum.h"

Servo M0;
Servo M1;
Servo M2;
Servo M3;
Servo M4;
Servo M5;
Servo M6;
Servo M7;

Servo servos[8] = { M0, M1, M2, M3, M4, M5, M6, M7 };
 int servo_pins[8] = { MAIN_M0,
                       MAIN_M1,
                       MAIN_M2,
                       MAIN_M3,
                       MAIN_M4,
                       MAIN_M5,
                       MAIN_M6,
                       MAIN_M7 };

String signal;
bool kill = false;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  for (int s = 0; s < 8; s++) {
    servos[s].attach(servo_pins[s]); // attach servos to their pins
  }
}

void loop() {
  int motor; // which motor is being set
  int velocity; // speed motor is set to
  if (kill) {
    Wire.beginTransmission(PWR); 
    Wire.write(E_STOP);
    Wire.endTransmission();
    delay(50);
  } else {
    if (Serial.available()) {
      signal = Serial.readString();
      if (signal[0] == E_STOP) {
          kill = true;
      } else if (signal.length() == 5 && signal[1] == ':') {
        motor = signal[0] - '0'; // subtract value for char 0
        velocity = signal.substring(2).toInt();
        servos[motor].write(velocity);
        delay(15);
        Serial.println(motor);
        Serial.println(velocity);
      }
    }
  }
}
