#include <Wire.h>
const int IMU_i2c_addr = 0x68; // IMU is ICM-42670-P
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

int loop_timer;
int temp;

int displaycount = 0;

void setup() {
  // put your setup code here, to run once:

  Wire.begin();
  setup_imc_42670_P_registers();
  for(int cal_int = 0; cal_int < 1000; cal_int ++){
    read_imc_42670_P_data();
    gyro_x_cal += gyro_x;

    gyro_y_cal += gyro_y;

    gyro_z_cal += gyro_z;

    delay(3);
  }

  // dividing the results by 1000 to get the average.
  gyro_x_cal /= 1000;
  gyro_y_cal /= 1000;
  gyro_z_cal /= 1000;

  Serial.begin(115200);

  loop_timer = micros();  

  Serial.print(" HELLO WORLD! ");
  delay(1000);
}

void loop() {
  // get acceleration and gyroscope data from IMU

   read_imc_42670_P_data();

  // subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;


  Serial.print("Gyro x : ");
  Serial.print(gyro_x);
  Serial.print(" Gyro y : ");
  Serial.print(gyro_y);
  Serial.print(" Gyro z : ");
  Serial.print(gyro_z);
  Serial.print(" acc x : ");
  Serial.print(acc_x);
  Serial.print(" acc y : ");
  Serial.print(acc_y);
  Serial.print(" acc z : ");
  Serial.println(acc_z);
// angle_pitch += gyro_x * 0.0000611;
// angle_roll += gyro_y * 0.0000611;
//
// angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
//
// angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);
//
//
// acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
//
// angle_pitch_acc = asin((float)acc_y/acc_total_vector)*57.296;
//
// angle_roll_acc = asin((float)acc_x/acc_total_vector)*-57.296;
//
// angle_pitch_acc -= 0.0;
//
// angle_pitch_acc -= 0.0;
//
// if(set_gyro_angles){
//  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
//
//  angle_roll = angle_roll * 0.9996 +angle_roll_acc * 0.0004;
// }
// else{
//  angle_pitch = angle_pitch_acc;
//
//  angle_roll = angle_roll_acc;
//
//  set_gyro_angles = true;
//  
// }
//
// angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
//
// angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;
//
// displaycount = displaycount + 1;
//
// if(displaycount > 100){
//  Serial.println(angle_pitch_output);
// }
  
}

void setup_imc_42670_P_registers(){
  //Start Communication with the IMU
  Wire.beginTransmission(IMU_i2c_addr);
  // select power management register:
  Wire.write(0x1F); //register: PWR_MGMT0

  // reset power management:
  Wire.write(0x50);

  //End Transmission
  Wire.endTransmission();

  /****************
   Let's configure the accelerometer accuracy
   ****************/
  Wire.beginTransmission(IMU_i2c_addr);
  //selected register
  Wire.write(0x21); // ACCEL_CONFIG0
  //set register
  Wire.write(0x06); // reset
  Wire.endTransmission();

   /****************
   Let's configure the accelerometer filter
   ****************/
  Wire.beginTransmission(IMU_i2c_addr);
  //selected register
  Wire.write(0x24); // ACCEL_CONFIG1
  //set register
  Wire.write(0x41); // reset
  Wire.endTransmission();

  
    /****************
   Let's configure the gyroscope accuracy
   ****************/
  Wire.beginTransmission(IMU_i2c_addr);
  //selected register
  Wire.write(0x20); // GYRO_CONFIG0
  //set register
  Wire.write(0x06); // reset
  Wire.endTransmission();

   /****************
   Let's configure the gyroscope filter
   ****************/
  Wire.beginTransmission(IMU_i2c_addr);
  //selected register
  Wire.write(0x23); // GYRO_CONFIG1
  //set register
  Wire.write(0x31); // reset
  Wire.endTransmission();
}

void read_imc_42670_P_data(){
   /****************
   Let's read some acceleration data
   ****************/
  Wire.beginTransmission(IMU_i2c_addr);
  //selected register
  Wire.write(0x0B); // ACCEL_DATA_X1

  // we don't set the acceleration data
  Wire.endTransmission(); 

  // request 12 bytes of data : 16 bits (2 bytes) for each : ACCEL_DATAX (high, low), ACCEL_DATAY, ACCEL_DATAZ, GYRO_DATAX, GYRO_DATAY, GYRO_DATAZ
  Wire.requestFrom(IMU_i2c_addr,12); 

  // wait until all the bytes are received
  while(Wire.available() < 12);
  acc_x = Wire.read()<<8|Wire.read();
  acc_y = Wire.read()<<8|Wire.read();
  acc_z = Wire.read()<<8|Wire.read();
  gyro_x = Wire.read()<<8|Wire.read();
  gyro_y = Wire.read()<<8|Wire.read();
  gyro_z = Wire.read()<<8|Wire.read();
}
