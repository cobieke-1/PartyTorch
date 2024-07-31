#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
int red = 9; //10
int green = 10; //9
int blue = 6;

// int rgbCheck[121][3] = {
// {0, 255, 0},
// {0, 253, 2},
// {0, 251, 4},
// {0, 249, 6},
// {0, 247, 9},
// {0, 244, 11},
// {0, 242, 13},
// {0, 240, 15},
// {0, 238, 17},
// {0, 236, 19},
// {0, 234, 21},
// {0, 232, 23},
// {0, 230, 26},
// {0, 227, 28},
// {0, 225, 30},
// {0, 223, 32},
// {0, 221, 34},
// {0, 219, 36},
// {0, 217, 38},
// {0, 215, 40},
// {0, 213, 43},
// {0, 210, 45},
// {0, 208, 47},
// {0, 206, 49},
// {0, 204, 51},
// {0, 202, 53},
// {0, 200, 55},
// {0, 198, 57},
// {0, 196, 60},
// {0, 193, 62},
// {0, 191, 64},
// {0, 189, 66},
// {0, 187, 68},
// {0, 185, 70},
// {0, 183, 72},
// {0, 181, 74},
// {0, 179, 77},
// {0, 176, 79},
// {0, 174, 81},
// {0, 172, 83},
// {0, 170, 85},
// {0, 168, 87},
// {0, 166, 89},
// {0, 164, 91},
// {0, 162, 94},
// {0, 159, 96},
// {0, 157, 98},
// {0, 155, 100},
// {0, 153, 102},
// {0, 151, 104},
// {0, 149, 106},
// {0, 147, 108},
// {0, 145, 111},
// {0, 142, 113},
// {0, 140, 115},
// {0, 138, 117},
// {0, 136, 119},
// {0, 134, 121},
// {0, 132, 123},
// {0, 130, 125},
// {0, 128, 128},
// {0, 125, 130},
// {0, 123, 132},
// {0, 121, 134},
// {0, 119, 136},
// {0, 117, 138},
// {0, 115, 140},
// {0, 113, 142},
// {0, 111, 145},
// {0, 108, 147},
// {0, 106, 149},
// {0, 104, 151},
// {0, 102, 153},
// {0, 100, 155},
// {0, 98, 157},
// {0, 96, 159},
// {0, 94, 162},
// {0, 91, 164},
// {0, 89, 166},
// {0, 87, 168},
// {0, 85, 170},
// {0, 83, 172},
// {0, 81, 174},
// {0, 79, 176},
// {0, 77, 179},
// {0, 74, 181},
// {0, 72, 183},
// {0, 70, 185},
// {0, 68, 187},
// {0, 66, 189},
// {0, 64, 191},
// {0, 62, 193},
// {0, 60, 196},
// {0, 57, 198},
// {0, 55, 200},
// {0, 53, 202},
// {0, 51, 204},
// {0, 49, 206},
// {0, 47, 208},
// {0, 45, 210},
// {0, 43, 213},
// {0, 40, 215},
// {0, 38, 217},
// {0, 36, 219},
// {0, 34, 221},
// {0, 32, 223},
// {0, 30, 225},
// {0, 28, 227},
// {0, 26, 230},
// {0, 23, 232},
// {0, 21, 234},
// {0, 19, 236},
// {0, 17, 238},
// {0, 15, 240},
// {0, 13, 242},
// {0, 11, 244},
// {0, 9, 247},
// {0, 6, 249},
// {0, 4, 251},
// {0, 2, 253},
// {0, 0, 255}};

#define OUTPUT_READABLE_YAWPITCHROLL


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    pinMode(red, OUTPUT);
    pinMode(green, OUTPUT);
    pinMode(blue, OUTPUT);
   
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
   
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

       // Colour selection
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            setLEDColour(ypr[0] * 180/M_PI);
        #endif
    }
}

void setLEDColour(int currPos){
  int redValue = 0; 
  int greenValue = 0;
  int blueValue = 0;
  int currCase = 0;
  if( 120 < currPos && currPos <= 180){

    //redValue should be 0 at currPos = 120 and 128 at currPos = 180
    redValue = map(currPos, 120, 180, 0, 128);
    greenValue = 0;
    //blueValue should be 255 at currPos = 120 and 128 at currPos = 180
    blueValue = map(currPos, 120, 180, 255, 128)

    // redValue = rgbCheck[currPos-121][2];
    // greenValue = rgbCheck[currPos-121][0];
    // blueValue = rgbCheck[currPos-121][1];
    // currCase = 1;

    //When

  }else if( 0 <= currPos && currPos <= 120){

    redValue = 0;
    //greenValue should be 255 at currPos = 0 and 0 at currPos = 120
    greenValue = map(currPos, 0, 120, 255, 0);
    //blueValue should be 0 at currPos = 0 and 255 at currPos = 120
    blueValue = map(currPos, 0, 120, 0, 255);

  //   redValue = rgbCheck[currPos][0];
  //   greenValue = rgbCheck[currPos][1];
  //   blueValue = rgbCheck[currPos][2];
  //  currCase = 2;
  }else if(-119 <= currPos && currPos < 0){

    blueValue = 0;
    //redValue should be 255 at currPos = -120 and 0 at currPos = 0
    redValue = map(currPos, 0, 120, 255, 0);
    //greenValue should be 0 at currPos = -120 and 255 at currPos = 0
    greenValue = map(currPos, 0, 120, 0, 255);
        
    // redValue = rgbCheck[currPos+119][1];
    // greenValue = rgbCheck[currPos+119][2];
    // blueValue = rgbCheck[currPos+119][0];
    // currCase = 3;
  }else if( -180 <= currPos && currPos <= -120){

    greenValue = 0;
    //redValue should be 128 at currPos = -180 and 255 at currPos = -120
    redValue = map(currPos, -180, -120, 128, 255);
    //blueValue should be 128 at currPos = -180 and 0 at currPos = -120
    blueValue = map(currPos, -180, -120, 128, 0);

    // redValue = rgbCheck[currPos+60+180][2];
    // greenValue = rgbCheck[currPos+60+180][0];
    // blueValue = rgbCheck[currPos+60+180][1];
    
    // currCase = 4;
  }
   
    analogWrite(red,redValue);
    analogWrite(green, greenValue);
    analogWrite(blue, blueValue);

//    Serial.print(currPos);
//    Serial.print("\t");
//    Serial.print(currCase);
//    Serial.print("\t");
//    Serial.print(redValue);
//    Serial.print("\t");
//    Serial.print(greenValue);
//    Serial.print("\t");
//    Serial.println(blueValue);

 // check if colour has been selected ( check linear acceleration)

 

}
