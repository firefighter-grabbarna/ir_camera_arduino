
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
    if (incomingByte == 'S')Serial.println("sensor_arduino");
    else if (isdigit(incomingByte)) { // check if the incoming byte is a digit
      int digit = incomingByte - '0'; // convert the incoming byte to an integer
      Serial.println(digit);
      return digit; // return the digit
    }
  }
  return -1; // return -1 if no digit was read
}


void setup(){
  Serial.begin(115200);
  ir_camera_setup();
  servo_setup();
}


static int state = 0;


void loop(){
  int to_send[6];
  int result;
  get_ir_values(to_send);
  listen_button();
  
  int new_state = read_serial();
  if(new_state != -1) {
    state = new_state;
  }

  Serial.println(state);

  switch(state){
    case 1:
      result = search_for_light(to_send);
      if(result == Result::SUCCESS){
        play_melody();
        state = 2;
        }
      else if(result == Result::FAILURE) state = 0;
      break;
    case 2:
      result = servo_follow_fire(to_send);
      if(result == Result::SUCCESS) state = 3;
      else if(result == Result::FAILURE) state = 1; // not implemented
      break;
    case 3:
      extinguish_fire(to_send);
      state = 0;
      break;
    case 4:
      print_data(to_send); // for debugging
      break;
    case 5:
      center_servos();    // for debugging
      break;
    default:
      break;
  }
}