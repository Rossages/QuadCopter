/* 
 *  Date: 23rd December 2017
 *  
 *  The following code has been copied from the MPU_6050_short_Ex code by Jeff Rowberg
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

This will be used to calibrate the Gyro so ensure that all offsets are consistent and 
accurate.


The whole code is not here because i do not need all the functionality that the MPU-6050 has
I have taken the following functionalities

Readable Yaw pitch Roll
Readable Real Acceleration
Readable World acceleration 

I Will use the following code to read my orrientation for the quadcopter which will allow me 
to have self stabalising and allow modeling for the flight control system.

Pins Required:

Im not a 100% sure how these are set! 
* 3.3V Pin
* SDA connected to pin A4
* SCL connected to pin A5
* Int connected to pin 2

 =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= 
 
 
 TODO: GET THE ACCELEROMETER WORKING 
 
 */
/*
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#include <stdbool.h>
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6) ************ CANT USE THIS PIN ALREADY TAKEN
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
#define BUF_SIZE 60
//static circBuf_t g_inBuffer;
*/

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int setteled = false;
int settlecount = 0;
// To tell if both minbuf and maxbuf have been updated in the same itterarion
int minflag = 0; 
int maxflag = 0;
float maxbuf;
float minbuf;
float Yawdiff;
int Settled = 0;



// This will set up the co-ordinate system 
// TODO: Maybe Change this to a class! Might be easier and more convienents
struct GyroMeas {
  
  float Yaw;
  float Pitch;
  float Roll;
  uint32_t Time;
  
};

struct AccelMeas {
  
    float Accel_x;
    float Accel_y;
    float Accel_z;
    
};

struct GyroMeas Gyro; // This sets up Gyro as a class of Measurements.
struct GyroMeas Gyroprev; // for previous measurements.

struct AccelMeas Accel; // This sets up Gyro as a class of Measurements.
struct AccelMeas Accelprev; // for previous measurements.

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

//void setup() {
 
    // join I2C bus (I2Cdev library doesn't do this automatically)
    
    /* #if checks whether the value is true (in the C and C++ sense of everything but 0) 
    and if so, includes the code until the closing #endif. 
    if not, that code is removed from the copy of the file given to the compiler prior 
    to compilation (but it has no effect on the original source code file). */
    
    /*#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    // TODO: CHANGE SERIAL TO 9600 WHEN I MERGE THIS WITH FLIGHT_CONTROLLER_01
    Serial.begin(115200); // was 115200

    // initialize device
    Serial.println(F("Initializing I2C for mpu6050..."));
    mpu.initialize();

    // verify connection
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(0); //was 220
    mpu.setYGyroOffset(0); // was 76
    mpu.setZGyroOffset(0); // was -85
    mpu.setZAccelOffset(1688); // 1688 factory default for my test chip //was 1788

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

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
    //initCircBuf(&g_inBuffer, BUF_SIZE);
}*/


void Measure_MPU() // This function will take all the measurements from the MPU6050 chip.
{

// if programming failed, don't try to do anything
    if (!dmpReady) return;
    
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        Serial.print("Wires have detached / something is broken");
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

      #ifdef OUTPUT_READABLE_YAWPITCHROLL

          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          
          // assinging the real values to variables to be compared and manipulated
          Gyro.Yaw = ypr[0]*180/M_PI;
          Gyro.Pitch = ypr[1]*180/M_PI;
          Gyro.Roll = ypr[2]*180/M_PI;

          
      #endif
      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
        
    } // End of the Reading statements.Prev Gyro

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

//void loop() {
void ImuCal(){   

    Measure_MPU(); // Make current measurement from Gyro
       
    //Serial.print ("yrp: ");
    Serial.print (Gyro.Yaw);
    Serial.print("\t");
    Serial.print (Gyro.Roll);
    Serial.print("\t");
    Serial.println (Gyro.Pitch);


    if (Settled != true) {
        if ((settlecount == BUF_SIZE || settlecount == 0)); { // Size of the Buffer, and hasnt settled
          // this means that 60 measurements have passed and we can check 
          //to see if the Yaw is stable now 
    
            if (settlecount == 0){
                minbuf = abs(Gyro.Yaw);
                minflag = 1;
                
            }
            
            if (settlecount == BUF_SIZE){
              maxbuf = abs(Gyro.Yaw); 
              // when we leave this statement we hit the i++ imediatly beinging i back to 0 :)
              settlecount = -1; 
              maxflag = 1;
            }
    
            if (minflag == 1 && maxflag == 1){
              // both measurements are from the same 'buffer itteration'
              Yawdiff = abs(maxbuf - minbuf);
              minflag = 0;
              maxflag = 0;

              if (Yawdiff < 0.20){
                Serial.print("\n\n *** Settled *** \n \n");
                Settled = true; // The Gyro has settled
              }
              
            }
        }
    }
    settlecount++;


    // Here goes the rest of the stuff for the motors and stuff. :) 
}



