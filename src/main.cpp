
#include <Arduino.h>
#include "melodies.h"
#include "states.h"
#include "ir_camera.h"
#include "audio.h"
#include "return_message.h"


/****************************************
 * HELPER FUNCTIONS
*****************************************/
int char_to_int(char character){
  return character - '0';
}


int read_serial() {
  
  if (Serial.available() > 0) { // check if there is data available in the serial buffer
    char incomingByte = Serial.read(); // read the incoming byte
    if (incomingByte == 'S') {
      Serial.println("sensor_arduino");
      while (Serial.read() != '\n');
    }
    else if (isdigit(incomingByte)) { // check if the incoming byte is a digit
      int digit = incomingByte - '0'; // convert the incoming byte to an integer
      // Serial.println(digit);
      return digit; // return the digit
    }
  }
  return -1; // return -1 if no digit was read
}


void setup(){
  Serial.begin(115200);
  ir_camera_setup();
  servo_setup();
  // long start = millis();
  // while (millis() - start > 500) {
  //   if (Serial.available()) {
  //     char c = 
  //   }
  // }
}


static int state = 0;

// todo
// 1. inplement function for buttons
// 2. print array all the time
// 3. timeout på follow fire


void loop(){
  static int to_send[6];
  int result;
  get_ir_values(to_send);

  center_servos();

  if(listen_button() == 2){
    state = 0; // if startbutton pressed, start robot
    play_melody();
    // Serial.println("2");
  }
  if(listen_button() == 1){
    state = 2; // if stopbutton pressed, stop robot
    // Serial.println("1");
    play_melody();
  }
  
  int prev_state = state;

  int new_state = read_serial();
  if(new_state != -1) {
    state = new_state;
    // Serial.println(state);
    switch (state) {
      case 0:
        break;
      case 1:
        break;
      case 2:
        break;
      case 3:
        Serial.print(prev_state); //print current state
        Serial.print(","); //print current state
        print_data(to_send); //print cordinates of candle
        state = prev_state;
        break;
      case 4:
          play_melody();
          state = prev_state;
          break;
      case 5:
        extinguish_fire();
        state = prev_state;
        break;
    }
  }

  // switch(state){
  //   case 1:
  //     result = search_for_light(to_send);
  //     if(result == Result::SUCCESS){
  //       play_melody();
  //       state = 2;
  //       }
  //     else if(result == Result::FAILURE) state = 0;
  //     break;
  //   case 2:
  //     result = servo_follow_fire(to_send);
  //     // if(result == Result::SUCCESS) ; // do nothing
  //     if(result == Result::FAILURE) state = 1; // not implemented
  //     break;
  //   case 3:
  //     extinguish_fire(to_send);
  //     state = 0;
  //     break;
  //   case 4:
  //       Serial.print(prev_state); //print current state
  //       Serial.print(","); //print current state
  //       print_data(to_send); //print cordinates of candle
  //     state = prev_state;
  //     break;
  //   case 5:
  //     center_servos();    // for debugging
  //     break;
  //   case 8:
  //     break;
  //   default:
  //     break;
  // }
}