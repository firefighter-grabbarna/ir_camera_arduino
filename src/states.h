#ifndef STATES_H
#define STATES_H

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

int servo_follow_fire(int* data);
int search_for_light(int* data);
int listen_button();

void calculate_candle_coords(int *data);
void extinguish_fire(int *data);
void reset_ir_servo_angles();
void center_servos();
void servo_setup();

#endif