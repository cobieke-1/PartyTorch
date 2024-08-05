#include <Wire.h>
const int IMU_i2c_addr = 0x68; // IMU is ICM-42670-P
long gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

int loop_timer;
int temp;

int counter = 0;

void setup() {
  // put your setup code here, to run once:

  Wire.begin();
  Serial.begin(115200);
  setup_icm_42670_P_registers();
//  Serial.print("done setup: ");
  printstate();
  for(int cal_int = 0; cal_int < 1000; cal_int ++){
    read_icm_42670_P_data();
   
    gyro_x_cal += gyro_x;

    gyro_y_cal += gyro_y;

    gyro_z_cal += gyro_z;

    delay(3);
  }
   // dividing the results by 1000 to get the average.
  gyro_x_cal /= 1000;
  gyro_y_cal /= 1000;
  gyro_z_cal /= 1000;

 

  loop_timer = micros();  

//  Serial.print(" HELLO WORLD! ");
  delay(1000);
}

void loop() {

//  Serial.print(" This is IMU code ");
//  Serial.println(counter++);

//   get acceleration and gyroscope data from IMU

   read_icm_42670_P_data();
   printstate();
  // subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;




}

void setup_icm_42670_P_registers(){
  //Start Communication with the IMU
  Wire.beginTransmission(0x68);
  // select power management register:
  Wire.write(0x1F); //register: PWR_MGMT0

  // reset power management:
  Wire.write(0x9F); //(0x70);

  //End Transmission
  Wire.endTransmission();
  delay(1);
  /****************
   Let's configure the accelerometer accuracy
   ****************/
  Wire.beginTransmission(0x68);
  //selected register
  Wire.write(0x21); // ACCEL_CONFIG0
  //set register
  Wire.write(0x90); // reset
  Wire.endTransmission();
  delay(1);

   /****************
   Let's configure the accelerometer filter
   ****************/
  Wire.beginTransmission(0x68);
  //selected register
  Wire.write(0x24); // ACCEL_CONFIG1
  //set register
  Wire.write(0x01); // reset
  Wire.endTransmission();
  delay(1);

  
    /****************
   Let's configure the gyroscope accuracy
   ****************/
  Wire.beginTransmission(0x68);
  //selected register
  Wire.write(0x20); // GYRO_CONFIG0
  //set register
  Wire.write(0x90); // tried for 100Hz at 2000dps
  Wire.endTransmission();
  delay(1);
   /****************
   Let's configure the gyroscope filter
   ****************/
  Wire.beginTransmission(0x68);
  //selected register
  Wire.write(0x23); // GYRO_CONFIG1
  //set register
  Wire.write(0x01); // reset
  Wire.endTransmission();
  delay(1);

  Wire.beginTransmission(0x68);
  //selected register
  Wire.write(0x02); // SIGNAL_PATH_RESET
  //set register
  Wire.write(0x04); // fifo flush
  Wire.endTransmission();
  delay(1);

  
}

void read_icm_42670_P_data(){
   /****************
   Let's read some acceleration data
   ****************/
  Wire.beginTransmission(0x68);

//  Wire.write(0x02);
//  
//  Wire.endTransmission();
//  Wire.requestFrom(0x68,1); 
//  int powMan = Wire.read();
//  
//
//  Serial.print("Power Management : ");
//  Serial.println(powMan);
  //selected register
  Wire.write(0x0B); // ACCEL_DATA_X1

  // we don't set the acceleration data
  Wire.endTransmission(); 

  // request 12 bytes of data : 16 bits (2 bytes) for each : ACCEL_DATAX (high, low), ACCEL_DATAY, ACCEL_DATAZ, GYRO_DATAX, GYRO_DATAY, GYRO_DATAZ
  Wire.requestFrom(0x68,12); 

  // wait until all the bytes are received
  while(Wire.available() < 12);
  acc_x =  Wire.read()<<8|Wire.read();
  acc_y =  Wire.read()<<8|Wire.read();
  acc_z =  Wire.read()<<8|Wire.read();
  gyro_x = Wire.read()<<8|Wire.read();
  gyro_y = Wire.read()<<8|Wire.read();
  gyro_z = Wire.read()<<8|Wire.read();
}

void printstate(){
  Serial.println("gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z");
  Serial.print(gyro_x);
  Serial.print(",");
  Serial.print(gyro_y);
  Serial.print(",");
  Serial.print(gyro_z);
  Serial.print(",");
  Serial.print(acc_x);
  Serial.print(",");
  Serial.print(acc_y);
  Serial.print(",");
  Serial.println(acc_z);

  delay(100);
}
