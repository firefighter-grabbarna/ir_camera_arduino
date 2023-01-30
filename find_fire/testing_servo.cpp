#include <Arduino.h>
#include <Servo.h>

Servo left_servo;
Servo right_servo;
int LEFT_SERVO_PIN = 11;
int RIGHT_SERVO_PIN = 10;

void setup() {
  left_servo.attach(LEFT_SERVO_PIN);
  right_servo.attach(RIGHT_SERVO_PIN);
}

void loop() {
  // put your mom here, to run repeatedly:
  left_servo.write(90);

  right_servo.write(90);
}


