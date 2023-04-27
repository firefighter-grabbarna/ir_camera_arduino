// Wii Remote IR sensor  test sample code  by kako
// modified output for Wii-BlobTrack program by RobotFreak
#include "ir_camera.h"

/****************************************
 * Pins
*****************************************/
int left_ir = 7;
int right_ir = 8;

int IRsensorAddress = 0xB0;
int slaveAddress;

byte data_buf[16];
int iter_val;

int Ix[4];
int Iy[4];
int s;


void write_2bytes(byte d1, byte d2){
    Wire.beginTransmission(slaveAddress);
    Wire.write(d1); Wire.write(d2);
    Wire.endTransmission();
}


void ir_setup_commands(){
    slaveAddress = IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI
    //pinMode(ledPin, OUTPUT);      // Set the LED pin as output
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


void ir_cummunication(int *output_arr, bool left_sensor_active){
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
    if (left_sensor_active) offset = 2;
    output_arr[offset] = Ix[0], 
    output_arr[1 + offset] = Iy[0];

    return;
}


void left_sensor_on(bool left_on){
    if (left_on){
      digitalWrite(right_ir, HIGH);
      digitalWrite(left_ir, LOW);
    } else {
      digitalWrite(right_ir, LOW);
      digitalWrite(left_ir, HIGH);
    }
  }


void print_data(int *data){
  int array_size = 6;
  for(int i = 0; i < array_size; i++){
    Serial.print(data[i]);
    if(i != (array_size -1)){
      Serial.print(",");
    }
  }
  Serial.println("");
}


void ir_camera_setup(){
  pinMode(left_ir, OUTPUT); 
  pinMode(right_ir, OUTPUT); 

  left_sensor_on(true);
  ir_setup_commands();

  left_sensor_on(false);
  ir_setup_commands();
}

void get_ir_values(int *output_arr){
  left_sensor_on(true);
  ir_cummunication(output_arr, true);

  left_sensor_on(false);
  ir_cummunication(output_arr, false);  
}


