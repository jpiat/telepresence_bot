

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"



#include "MPU6050_6Axis_MotionApps20.h"



#define SET_P 25.0
#define SET_I 1.0
#define SET_D 1.0

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu ; // <-- use for AD0 high

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

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
long int gyro[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



//robot variables
char bot_ready = 0 ;
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}




struct pid_state{
   float error ;
   float integral ;
   float derivative ;
   float P ;
   float I ;
   float D ;
   char init_derivative ;
};




struct pid_state current_state ;


float init_pid(struct pid_state * state, float p, float i, float d){
  state->error = 0. ;
  state-> integral = 0.;
  state-> derivative = 0.;
  state->P = p ;
  state->I = i ;
  state->D = d ;
  state->init_derivative = 0 ;
}

float update_pid(struct pid_state * state, float theta_meas, float theta_cons){
    
  float new_error = theta_cons - theta_meas ;
  state->derivative = new_error - state->error ;
  state->error = new_error ;
  state->integral += new_error;
  if(state->integral > 50) state->integral = 50 ;
    if(state->integral < -50) state->integral = -50 ;

  float pid_output = state->P * state->error ; 
  pid_output += state->I * state->integral ;
  if(state->derivative){
    pid_output += state->D * state->derivative ;
  }
  state->init_derivative = 1 ;

  Serial.print(state->error);
  Serial.print(" : ");
  Serial.print(state->integral);
  Serial.print(" : ");
  Serial.print(state->derivative);
  Serial.print(" ->  ");
  Serial.println(pid_output);
  
  return pid_output ;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize(100); //200Hz update rate

    // supply your own gyro offsets here, scaled for min sensitivity
    /*mpu.setXGyroOffset(785);
    mpu.setYGyroOffset(165);
    mpu.setZGyroOffset(-272);
    mpu.setZAccelOffset(1788);*/ // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      //if configuration of IMU failed, go to blink of death
        while(1){
          digitalWrite(LED_PIN, HIGH);
          delay(200);
          digitalWrite(LED_PIN, LOW);
          delay(200);
        }
    }

    init_pid(&current_state, SET_P, SET_I, SET_D);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
unsigned int init_measurement = WAIT_INIT_SAMPLES ;
void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) ;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if(fifoCount == 0) return  ; // no data available

    digitalWrite(LED_PIN, HIGH);
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;


        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
         float deg_angle = euler[1] * 180/M_PI ;
        if(bot_ready){
          float motor_drive = update_pid(&current_state, deg_angle, 0.f);
          if(motor_drive > 255) motor_drive = 255 ;
          if(motor_drive < -255) motor_drive = -255 ;
          /*Serial.print(deg_angle);
          Serial.print(" : ");*/
          //Serial.println(motor_drive);
        }else{
          //Serial.println(deg_angle);
          if((deg_angle > 0.0 && deg_angle < 2.0 || deg_angle < 0.0 && deg_angle > -2.0 ) && init_measurement == 0){
            bot_ready = 1 ;
          }
          if(init_measurement > 0) init_measurement -- ;
        }
       
    }
    digitalWrite(LED_PIN, LOW);

}
