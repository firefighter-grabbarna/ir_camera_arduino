// Inspired by:
// * Wii Remote IR sensor  test sample code  by kako
// * modified output for Wii-BlobTrack program by RobotFreak

#include "states.h"
#include "return_message.h"

/****************************************
 * Pins
*****************************************/
const int RED_BUTTON_PIN = 12;
const int GREEN_BUTTON_PIN = 13;

const int LEFT_SERVO_PIN = 11;
const int RIGHT_SERVO_PIN = 9;
const int ARM_SERVO_PIN = 10;


/****************************************
 * Servo stuff
*****************************************/
double left_servo_angle = 90;
double right_servo_angle = 90;
Servo left_servo;
Servo right_servo;
Servo arm_servo;

int search_for_light_ctr = 0;


void servo_setup(){
  // Set up the servo
  arm_servo.attach(ARM_SERVO_PIN);
  arm_servo.write(165);
  left_servo.attach(LEFT_SERVO_PIN);
  left_servo.write(90);
  right_servo.attach(RIGHT_SERVO_PIN);
  right_servo.write(90);
}


/**
 * @brief Tries to extinguish the fire by rotating
 * the servo controlling the extinguishing mechanism.
 * 
 * @param data Used to reset x and y coordinates of candle.
 */
void extinguish_fire(int* data){
  const int NO_DATA = 1023;
  const int MIN_ANGLE = 20;
  const int MAX_ANGLE = 165;
  data[4] = NO_DATA;
  data[5] = NO_DATA;
  arm_servo.write(MIN_ANGLE);
  delay(2000);
  arm_servo.write(MAX_ANGLE);
  delay(2000);
}

/**
 * @brief Follows the light with the cameras by rotating the servos
 * holding the ir-cameras
 * 
 * @param data Array storing x and y values given by ir-cameras (and more)
 */
// unsigned long prev_time = 0;
// double speed_left = 0;
// double speed_right = 0;  
int servo_follow_fire(int* data){ //TODO try different values for MARGIN to get better accuracy
  const double MIN_ANGLE = 10;
  // const double MAX_ANGLE = 170;
  const int MIDDLE = 512;
  const int NO_DATA = 1023;
  const int MARGIN = 30;
  const int LEFT_SENSOR_X_VALUE = data[0];
  const int RIGHT_SENSOR_X_VALUE = data[2];
  int return_message = Result::RUNNING;

  bool left_servo_incorrect = (LEFT_SENSOR_X_VALUE < MIDDLE - MARGIN || LEFT_SENSOR_X_VALUE > MIDDLE + MARGIN)
                               && LEFT_SENSOR_X_VALUE != NO_DATA;
  bool right_servo_incorrect = (RIGHT_SENSOR_X_VALUE < MIDDLE - MARGIN || RIGHT_SENSOR_X_VALUE > MIDDLE + MARGIN) 
                                && RIGHT_SENSOR_X_VALUE != NO_DATA;
  const double Kp_left = 0.005;
  const double Kp_right = 0.003;
  // const double Ki = 0.5;
  double speed_left = Kp_left * (LEFT_SENSOR_X_VALUE -  MIDDLE); 
  double speed_right = Kp_right * (RIGHT_SENSOR_X_VALUE - MIDDLE); 

  // unsigned long delta_time = millis() - prev_time;
  // double error_left = (LEFT_SENSOR_X_VALUE -  MIDDLE);
  // double error_right = (RIGHT_SENSOR_X_VALUE -  MIDDLE);
  // speed_left = Ki*(speed_left + error_left * delta_time) + Kp_left * (LEFT_SENSOR_X_VALUE -  MIDDLE);
  // speed_right = Ki*(speed_right + error_right * delta_time) + Kp_right * (RIGHT_SENSOR_X_VALUE - MIDDLE);
  // prev_time = millis();
  /*

  create i regulator
  double error_left = (LEFT_SENSOR_X_VALUE -  MIDDLE);
  double error_right = (RIGHT_SENSOR_X_VALUE -  MIDDLE);
  double prev_left += speed_left + error_left * delta_time;
  double prev_right += speed_right + error_right * delta_time;
  */


  // Adjust left servo
  double new_angle;
  if (left_servo_incorrect){
    new_angle = max(left_servo_angle + speed_left, MIN_ANGLE);
    left_servo_angle = new_angle;
  }

  // Adjust right servo
  if(right_servo_incorrect){
    new_angle = max(right_servo_angle + speed_right, MIN_ANGLE);
    right_servo_angle = new_angle;
  }

  left_servo.write((int)left_servo_angle);
  right_servo.write((int)right_servo_angle);
  
  calculate_candle_coords(data);

  // If the light is close enough to be extinguished 
  int x_coord = data[4];
  int y_coord = data[5]; 
  const int MAX_DISTANCE_Y = 180, MAX_DISTANCE_X = 30;
  if (y_coord < MAX_DISTANCE_Y && abs(x_coord+50) < MAX_DISTANCE_X){
    // return_message = 1;
    return_message = Result::SUCCESS;
  }
  return return_message;
}


/**
 * @brief Searches for the candle
 * 
 */
int search_for_light(int* data){
  
  const double ROTATION_SPEED = 0.5;  
  const int NO_DATA = 1023;
  const int MAX_ANGLE = 170;
  bool left_found = false;
  bool right_found = false;
  int return_message = Result::RUNNING;
  // Sweep left servo until light found
  // or max angle 
  const int LEFT_SENSOR_X_VALUE = data[0];
  // Serial.println(left_servo_angle);

  if (left_servo_angle < MAX_ANGLE){
    if (LEFT_SENSOR_X_VALUE == NO_DATA){
      
      left_servo_angle += ROTATION_SPEED;
      left_servo.write((int)left_servo_angle);
    }
    else{
      left_found = true;
    }
  }

  // Sweep right servo until light found
  // or max angle hit
  const int RIGHT_SENSOR_X_VALUE = data[2];
  if (right_servo_angle < MAX_ANGLE){
    if (RIGHT_SENSOR_X_VALUE == NO_DATA){
      right_servo_angle += ROTATION_SPEED;
      right_servo.write((int)right_servo_angle);
    }
    else{
      right_found = true;
    }
  }

  // If both found, check multiple times
  // to avoid false positives
  const int TIMES_TO_CHECK = 10;
  const int ERROR_MARGIN_DEG = 2;
  if (left_found && right_found){
    search_for_light_ctr++;

    if(search_for_light_ctr > TIMES_TO_CHECK){
      // return_message = 1;
      return_message = Result::SUCCESS;
      search_for_light_ctr = 0;
    }
    return return_message;
  }
  else if (right_servo_angle >= (MAX_ANGLE - ERROR_MARGIN_DEG) || left_servo_angle >= (MAX_ANGLE - ERROR_MARGIN_DEG)){
    // Why?
    if (right_servo_angle >= 160 && left_servo_angle >= 160){
      
      // return_message = 2;
      return_message = Result::FAILURE;
    }   
    reset_ir_servo_angles();
  }
  return return_message;
}


/**
 * @brief listens for buttonklick
 * 
 */
int listen_button() {
  int button_pressed = 0;
  if (digitalRead(RED_BUTTON_PIN) == HIGH) {
    button_pressed = 1;
  }
  if(digitalRead(GREEN_BUTTON_PIN) == HIGH){
    button_pressed = 2;
  }
  return button_pressed;
}

/****************************************
 * HELPER FUNCTIONS
*****************************************/

/**
 * @brief Resets the angles of the left and right servo
 * 
 */
void reset_ir_servo_angles(){
  const int MIN_ANGLE = 20;
  right_servo_angle = MIN_ANGLE;
  right_servo.write(right_servo_angle);
  left_servo_angle = MIN_ANGLE;
  left_servo.write(left_servo_angle);
}


/**
 * @brief Calculates the coordinates of the candle
 * 
 * @param data Array where the coordinates (and more) are stored
 */
void calculate_candle_coords(int *data){
  const int DIST_ORIGIN_SENSORS_MM = 190/2;
  const int SERVO_OFFSET = 3;

  // Convert degrees to radians and mirror right servo angle
  double left_servo_rad = (left_servo_angle) * PI / 180 + SERVO_OFFSET;
  double right_servo_rad = -(right_servo_angle) * PI / 180 + SERVO_OFFSET;

  // Parameter in line expressed in parametric form
  double parameter = 2 * DIST_ORIGIN_SENSORS_MM * sin(left_servo_rad) / sin(left_servo_rad + right_servo_rad);
  
  // Coordinates of the light assuming origin in-between the two IR-cameras
  double x_coord = -cos(right_servo_rad) * parameter + DIST_ORIGIN_SENSORS_MM;
  double y_coord = sin(right_servo_rad) * parameter;
  
  data[4] = (int) x_coord;
  data[5] = (int) y_coord;
}


void center_servos(){
  const int MIDDLE = 90;
  left_servo.write(MIDDLE);
  right_servo.write(MIDDLE);
}


