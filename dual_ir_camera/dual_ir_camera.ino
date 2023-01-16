// Wii Remote IR sensor  test sample code  by kako
// modified output for Wii-BlobTrack program by RobotFreak

#include <Wire.h>

int IRsensorAddress = 0xB0;
int slaveAddress;
int ledPin = 13;
boolean ledState = false;
byte data_buf[16];
int i;

bool ir_bool = true;
int left_ir = 7;
int right_ir = 6;
int sensor_values[4];
int outside_counter;

int Ix[4];
int Iy[4];
int s;

void Write_2bytes(byte d1, byte d2)
{
    Wire.beginTransmission(slaveAddress);
    Wire.write(d1); Wire.write(d2);
    Wire.endTransmission();
}

void left_sensor_on(bool left_on){
    if(left_on){
      digitalWrite(left_ir, HIGH);
      digitalWrite(right_ir, LOW);
    } else {
      digitalWrite(left_ir, LOW);
      digitalWrite(right_ir, HIGH);
    }
  
  }

void ir_setup(){
    pinMode(left_ir, OUTPUT); 
    pinMode(right_ir, OUTPUT); 
    digitalWrite(left_ir, HIGH);
  
    slaveAddress = IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI
    Serial.begin(38400);
    pinMode(ledPin, OUTPUT);      // Set the LED pin as output
    Wire.begin();
    // IR sensor initialize
    Write_2bytes(0x30,0x01); delay(10);
    Write_2bytes(0x30,0x08); delay(10);
    Write_2bytes(0x06,0x90); delay(10);
    Write_2bytes(0x08,0xC0); delay(10);
    Write_2bytes(0x1A,0x40); delay(10);
    Write_2bytes(0x33,0x33); delay(10);
    delay(100);
  }

void ir_camera_loop()
{
    ledState = !ledState;
    if (ledState) { digitalWrite(ledPin,HIGH); } else { digitalWrite(ledPin,LOW); }

    //IR sensor read
    Wire.beginTransmission(slaveAddress);
    Wire.write(0x36);
    Wire.endTransmission();

    Wire.requestFrom(slaveAddress, 16);        // Request the 2 byte heading (MSB comes first)
    for (i=0;i<16;i++) { data_buf[i]=0; }
    i=0;
    while(Wire.available() && i < 16) { 
        data_buf[i] = Wire.read();
        i++;
    }

    Ix[0] = data_buf[1];
    Iy[0] = data_buf[2];
    s   = data_buf[3];
    Ix[0] += (s & 0x30) <<4;
    Iy[0] += (s & 0xC0) <<2;

    Ix[1] = data_buf[4];
    Iy[1] = data_buf[5];
    s   = data_buf[6];
    Ix[1] += (s & 0x30) <<4;
    Iy[1] += (s & 0xC0) <<2;

    Ix[2] = data_buf[7];
    Iy[2] = data_buf[8];
    s   = data_buf[9];
    Ix[2] += (s & 0x30) <<4;
    Iy[2] += (s & 0xC0) <<2;

    Ix[3] = data_buf[10];
    Iy[3] = data_buf[11];
    s   = data_buf[12];
    Ix[3] += (s & 0x30) <<4;
    Iy[3] += (s & 0xC0) <<2;

    // Since we only use the first value
    // the zero is a place-holder
    int output_array[2] = {Ix[0], Iy[0]};

     


    // int offset = 0; 
    // if(ir_bool) offset = 2;
    // if(sensor_values[offset] != 1023 && sensor_values[offset] != 0){
    //   sensor_values[offset] = int(Ix[0]);
    //   outside_counter=0;
    // }
    // if(sensor_values[offset+1] != 1023 && sensor_values[offset+1] != 0){
    //   sensor_values[offset+1] = int(Iy[1]);
    //   outside_counter=0;
    // }
    // if(outside_counter > 20){
    //   sensor_values[offset] = 0;
    //   sensor_values[offset+1] = 0;
    // }
    // outside_counter++;
    
    // Serial.print(sensor_values[offset]);
    // Serial.print(",");
    // Serial.print(sensor_values[offset+1]);

    Serial.print(int(Ix[0]));
    Serial.print(",");
    Serial.print(int(Iy[0]));
    return;




    // for(i=0; i<4; i++)
    // {
    //   if (Ix[i] < 1000)
    //     Serial.print(" ");
    //   if (Ix[i] < 100)  
    //     Serial.print(" ");
    //   if (Ix[i] < 10)  
    //     Serial.print(" ");
    //   Serial.print( int(Ix[i]) );
    //   Serial.print(",");
    //   if (Iy[i] < 1000)
    //     Serial.print(" ");
    //   if (Iy[i] < 100)  
    //     Serial.print(" ");
    //   if (Iy[i] < 10)  
    //     Serial.print(" ");
    //   Serial.print( int(Iy[i]) );
    //   if (i<3)
    //     Serial.print(",");
    // }
    // Serial.println(""); 
}

void send_data(int *data[]){
  //Serial.println("hello");
  for(int i = 0; i < 4; i++){
    Serial.print(*data[i]);
    Serial.print(",");
  }
  Serial.println("");
}

void setup()
{
  left_sensor_on(true);
  ir_setup();
  left_sensor_on(false); // turns on right sensor
  ir_setup();
  left_sensor_on(true);
}

void loop()
{
  int to_send[4];

  // First sensor
  left_sensor_on(ir_bool);  // change active sensor
  ir_camera_loop(); // do the communication
  ir_bool = !ir_bool;

  Serial.print(",");

  // secound sensor
  left_sensor_on(ir_bool);  // change active sensor
  ir_camera_loop(); // do the communication
  ir_bool = !ir_bool;

  // //send_data(*to_send);  
  // for(int i = 0; i < 4; i++){
  //   Serial.print(to_send[i]);
  //   Serial.print(",");
  // }
  Serial.println("");

  delay(10);
}