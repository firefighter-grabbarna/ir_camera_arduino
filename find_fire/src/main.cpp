// Wii Remote IR sensor  test sample code  by kako
// modified output for Wii-BlobTrack program by RobotFreak

#include <Wire.h>
#include <Arduino.h>
#include <Servo.h>

int IRsensorAddress = 0xB0;
int slaveAddress;
int ledPin = 13;
boolean ledState = false;
byte data_buf[16];
int iter_val;

bool ir_bool = true;
int left_ir = 7;
int right_ir = 6;
int sensor_values[4]; // Remove?
int outside_counter; // Remove?

double left_servo_angle = 90;
double right_servo_angle = 90;
Servo left_servo;
Servo right_servo;
Servo arm_servo;
int LEFT_SERVO_PIN = 11;
int RIGHT_SERVO_PIN = 10;
int ARM_SERVO_PIN = 9;

int state = 0;
int search_for_light_ctr = 0;

int Ix[4];
int Iy[4];
int s;

/**
 * @brief Writes 2 bytes to slaveAddress.
 * 
 * @param d1 Byte 1
 * @param d2 Byte 2
 */
void write_2bytes(byte d1, byte d2){
    Wire.beginTransmission(slaveAddress);
    Wire.write(d1); Wire.write(d2);
    Wire.endTransmission();
}

/**
 * @brief Toggles which sensor is active
 * 
 * @param left_on True when left active, false when right active.
 */
void left_sensor_on(bool left_on){
    if (left_on){
      digitalWrite(left_ir, HIGH);
      digitalWrite(right_ir, LOW);
    } else {
      digitalWrite(left_ir, LOW);
      digitalWrite(right_ir, HIGH);
    }
  }

/**
 * @brief Setup for an ir-camera.
 */
void ir_setup(){
    pinMode(left_ir, OUTPUT); 
    pinMode(right_ir, OUTPUT); 
    digitalWrite(left_ir, HIGH); // Can only handle one at a time
  
    slaveAddress = IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI
    pinMode(ledPin, OUTPUT);      // Set the LED pin as output
    Wire.begin();
    // IR sensor initialize
    // No one knows why these hex numbers work, probably black magic.
    // Don't touch for any reason, you have been warned.
    write_2bytes(0x30,0x01); delay(10); 
    write_2bytes(0x30,0x08); delay(10);
    write_2bytes(0x06,0x90); delay(10);
    write_2bytes(0x08,0xC0); delay(10);
    write_2bytes(0x1A,0x40); delay(10);
    write_2bytes(0x33,0x33); delay(10);
    delay(100);
  }

/**
 * @brief Communicates with the active camera 
 * 
 * @param output_arr The data to send to the ir-camera
 */
void ir_camera_loop(int *output_arr){
    ledState = !ledState;
    if (ledState) { digitalWrite(ledPin,HIGH); } else { digitalWrite(ledPin,LOW); }

    //IR sensor read
    Wire.beginTransmission(slaveAddress);
    Wire.write(0x36);
    Wire.endTransmission();

    Wire.requestFrom(slaveAddress, 16);        // Request the 2 byte heading (MSB comes first)
    for (iter_val = 0; iter_val < 16; iter_val++) { 
      data_buf[iter_val] = 0; 
    }
    
    iter_val = 0;
    while(Wire.available() && iter_val < 16) { 
        data_buf[iter_val] = Wire.read();
        iter_val++;
    }

    // More black magic
    Ix[0] = data_buf[1];
    Iy[0] = data_buf[2];
    s = data_buf[3];
    Ix[0] += (s & 0x30) << 4;
    Iy[0] += (s & 0xC0) << 2;

    Ix[1] = data_buf[4];
    Iy[1] = data_buf[5];
    s = data_buf[6];
    Ix[1] += (s & 0x30) << 4;
    Iy[1] += (s & 0xC0) << 2;

    Ix[2] = data_buf[7];
    Iy[2] = data_buf[8];
    s = data_buf[9];
    Ix[2] += (s & 0x30) << 4;
    Iy[2] += (s & 0xC0) << 2;

    Ix[3] = data_buf[10];
    Iy[3] = data_buf[11];
    s = data_buf[12];
    Ix[3] += (s & 0x30) << 4;
    Iy[3] += (s & 0xC0) << 2;

    // The left of the sensor and right sensor occupy diffrent parts of the array
    int offset = 0;
    if (ir_bool) offset = 2;
    output_arr[offset] = Ix[0], 
    output_arr[1 + offset] = Iy[0];

    return;
}

/**
 * @brief Sends data over serial
 * 
 * @param data The array of data to be sent
 */
void send_data(int *data){
  //int array_size = sizeof(data)/sizeof(int);
  int array_size = 6;
  for(int i = 0; i < array_size; i++){
    Serial.print(data[i]);
    if(i != (array_size -1)){
      Serial.print(",");
    }
  }
  Serial.println("");
}

/**
 * @brief Calculates the coordinates of the candle
 * 
 * @param data Array where the coordinates (and more) are stored
 */
void calculate_candle_coords(int *data){
  const int DIST_ORIGIN_SENSORS_MM = 85;

  // Convert degrees to radians and mirror right servo angle
  double left_servo_rad = (left_servo_angle) * PI / 180;
  double right_servo_rad = -(right_servo_angle) * PI / 180;

  // Parameter in line expressed in parametric form
  double parameter = 2 * DIST_ORIGIN_SENSORS_MM * sin(left_servo_rad) / sin(left_servo_rad + right_servo_rad);
  
  // Coordinates of the light assuming origin in-between the two IR-cameras
  double x_coord = -cos(right_servo_rad) * parameter + DIST_ORIGIN_SENSORS_MM;
  double y_coord = sin(right_servo_rad) * parameter;
  
  data[4] = (int) x_coord;
  data[5] = (int) y_coord;

  // // If the light is close enough to be extinguished 
  // const int MAX_DISTANCE_Y = 200, MAX_DISTANCE_X = 30;
  // if (y_coord < MAX_DISTANCE_Y && abs(x_coord) < MAX_DISTANCE_X){
  //   state = 3; // Put out fire
  // }
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
void servo_follow_fire(int* data){ //TODO try different values for MARGIN to get better accuracy
  const double MIN_ANGLE = 10;
  const double MAX_ANGLE = 170;
  const int MIDDLE = 512;
  const int NO_DATA = 1023;
  const int MARGIN = 30;
  const int LEFT_SENSOR_X_VALUE = data[0];
  const int RIGHT_SENSOR_X_VALUE = data[2];

  bool left_servo_incorrect = (LEFT_SENSOR_X_VALUE < MIDDLE - MARGIN || LEFT_SENSOR_X_VALUE > MIDDLE + MARGIN)
                               && LEFT_SENSOR_X_VALUE != NO_DATA;
  bool right_servo_incorrect = (RIGHT_SENSOR_X_VALUE < MIDDLE - MARGIN || RIGHT_SENSOR_X_VALUE > MIDDLE + MARGIN) 
                                && RIGHT_SENSOR_X_VALUE != NO_DATA;
  const double Kp = 0.005;
  double speed_left = Kp * (LEFT_SENSOR_X_VALUE - MIDDLE); 
  double speed_right = Kp * (RIGHT_SENSOR_X_VALUE - MIDDLE); 

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
}

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
 * @brief Searches for the candle
 * 
 */
void search_for_light(int* data){
  const double ROTATION_SPEED = 0.5;  
  const int NO_DATA = 1023;
  const int MAX_ANGLE = 170;
  bool left_found = false;
  bool right_found = false;

  // Sweep left servo until light found
  // or max angle hit
  const int LEFT_SENSOR_X_VALUE = data[0];
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
      state = 2; // Follow light state
      Serial.println("found");
      search_for_light_ctr = 0;
    }
    return;
  }
  else if (right_servo_angle >= (MAX_ANGLE - ERROR_MARGIN_DEG) || left_servo_angle >= (MAX_ANGLE - ERROR_MARGIN_DEG)){
    // Why?
    if (right_servo_angle >= 160 && left_servo_angle >= 160){
      state = 0;
      Serial.println("not found");
    }
    
    reset_ir_servo_angles();
    delay(1000);
  }
}

/**
 * @brief Converts a char to an int
 * 
 * @param character The character to convert
 * @return int 
 */
int char_to_int(char character){
  return character - '0';
}

void setup(){
  Serial.begin(115200);
  left_servo.attach(LEFT_SERVO_PIN);
  right_servo.attach(RIGHT_SERVO_PIN);
  arm_servo.attach(ARM_SERVO_PIN);
  arm_servo.write(165);
  
  left_sensor_on(true);
  ir_setup();
  left_sensor_on(false); // turns on right sensor
  ir_setup();
  left_sensor_on(true);
}

void loop(){
  // 0: left x, 1: left y, 2: right x, 3: right y, 4: candle x, 5: candle y 
  int to_send[6];

  int prev_state = state;

  if (Serial.available()){
    String input = Serial.readStringUntil('\n');
    if (input[0] >= '0' && input[0] <= '9'){
      char state_command = input[0];
      state = char_to_int(state_command);
    }
  }
  //Serial.println(state);
  
  // State 0 = Nothing
  // State 1 = Search for light
  // State 2 = Move to light
  //state = 1;
  switch(state){
    case 1:
      search_for_light(to_send);
      break;
    case 2:
      servo_follow_fire(to_send);
      break;
    case 3:
      extinguish_fire(to_send);
      Serial.println("done");
      state = 0;
      break;
    case 4:
      send_data(to_send);
      state = prev_state;
      break;
    case 5:
      right_servo_angle = 90;
      left_servo_angle = 90;
      right_servo.write((int)right_servo_angle);
      left_servo.write((int)left_servo_angle);
      state = prev_state;
      break;
    default:
      break;
  }
  /*
  if (state == 0);
  else if (state == 1) search_for_light(to_send);
  else if (state == 2) servo_follow_fire(to_send);
  else if (state == 3) extinguish_fire(to_send);
  */

  // Left sensor
  left_sensor_on(ir_bool); // change active sensor true is left, False is right
  ir_camera_loop(to_send); // do the communication
  ir_bool = !ir_bool;

  // Right sensor
  left_sensor_on(ir_bool); // change active sensor true is left, False is right
  ir_camera_loop(to_send); // do the communication
  ir_bool = !ir_bool;

  //servo_follow_fire(to_send);

  delay(10);
}