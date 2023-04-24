
/*
  In this example, the Arduino simulates the sampling of a sinusoidal 1000 Hz
  signal with an amplitude of 100, sampled at 5000 Hz. Samples are stored
  inside the vReal array. The samples are windowed according to Hamming
  function. The FFT is computed using the windowed samples. Then the magnitudes
  of each of the frequencies that compose the signal are calculated. Finally,
  the frequency with the highest peak is obtained, being that the main frequency
  present in the signal.
*/
#include "audio.h"

const int BUZZER = 3;


void microphone_iteration()
{

  /*
  These values can be changed in order to evaluate the functions
  */
  const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
  const double samplingFrequency = 2 * 4096;
  /*
  These are the input and output vectors
  Input vectors receive computed results from FFT
  */
  double vReal[samples];
  double vImag[samples];



  unsigned int sampling_period_us = round(1000000*(1.0/samplingFrequency));
  arduinoFFT FFT2 = arduinoFFT(); /* Create FFT object */
  /* Build raw data */
   
  // Serial.println("Computed magnitudes: ");
   unsigned long microseconds = micros();
  for(unsigned int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(A0);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }

  /* Print the results of the simulated sampling according to time */
  FFT2.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
  FFT2.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT2.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */

  double x = FFT2.MajorPeak(vReal, samples, samplingFrequency);
  Serial.println(x);
}


/**
 * @brief Plays a melody
 * 
 */
void play_melody() {
  // change this to make the song slower or faster
  int tempo = 150;
  int notes = sizeof(melody) / sizeof(melody[0]) / 2;

  // this calculates the duration of a whole note in ms
  int wholenote = (60000 * 2) / tempo;
  int divider = 0, noteDuration = 0;

  // iterate over the notes of the melody. 
  // Remember, the array is twice the number of notes (notes + durations)
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(BUZZER, melody[thisNote], noteDuration*0.9);

    // Wait for the specief duration before playing the next note.
    delay(noteDuration);
    
    // stop the waveform generation before the next note.
    noTone(BUZZER);
  }
}