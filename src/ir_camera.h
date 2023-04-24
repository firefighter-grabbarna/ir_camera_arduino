
#ifndef IR_CAMERA_H
#define IR_CAMERA_H

#include <Arduino.h>
#include <Wire.h>

void ir_setup_commands();
void ir_cummunication(int *output_arr);
void left_sensor_on(bool left_on);
void print_data(int *data);
void write_2bytes(byte d1, byte d2);
void get_ir_values(int *output_arr);
void ir_camera_setup();

#endif