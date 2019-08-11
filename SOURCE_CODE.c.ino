#include "head.h"
#include "algorithm.h"

void setup() {
  // put your setup code here, to run once:
  pinMode(DC_PWM_PIN, OUTPUT);
  pinMode(DC_DIR_PIN, OUTPUT);
  pinMode(STEER_PWM_PIN, OUTPUT);
  pinMode(STEER_DIR_PIN, OUTPUT);
  Serial.begin(9600);
  Algorithm_init();
}

void loop() {
  // put your main code here, to run repeatedly:
  Algorithm_update();
}
