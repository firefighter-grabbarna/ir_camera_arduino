#ifndef AUDIO_H
#define AUDIO_H

#include "arduinoFFT.h"
#include "melodies.h"

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

#define melody short_melody

bool microphone_iteration();
void play_melody();

#endif