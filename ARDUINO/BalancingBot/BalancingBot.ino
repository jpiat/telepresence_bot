

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"



#include "MPU6050_6Axis_MotionApps20.h"


//define for motor control pins
#define MOT_A_EN_A_PIN 8
#define MOT_A_EN_B_PIN 7
#define MOT_A_PWM_PIN 6

#define MOT_B_EN_A_PIN 3
#define MOT_B_EN_B_PIN 4
#define MOT_B_PWM_PIN 5

#define MC33886

void setMotorASpeedVNH2SP30(float motor_speed){
  if(motor_speed < 0.005  && motor_speed > -0.005){ //brake
      digitalWrite(MOT_A_EN_A_PIN, LOW);
      digitalWrite(MOT_A_EN_B_PIN, LOW);
  }else if(motor_speed > 0){ //foward
      digitalWrite(MOT_A_EN_A_PIN, HIGH);
      digitalWrite(MOT_A_EN_B_PIN, LOW);
   }else{//backward
     digitalWrite(MOT_A_EN_A_PIN, LOW);
     digitalWrite(MOT_A_EN_B_PIN, HIGH);
   } 
   motor_speed = motor_speed > 0 ? motor_speed * 255 : (-motor_speed * 255) ;
   analogWrite(MOT_A_PWM_PIN, motor_speed);
}

void setMotorBSpeedVNH2SP30(float motor_speed){
  if(motor_speed < 0.005  && motor_speed > -0.005){ //brake
      digitalWrite(MOT_B_EN_A_PIN, LOW);
      digitalWrite(MOT_B_EN_B_PIN, LOW);
  }else if(motor_speed > 0){ //foward
      digitalWrite(MOT_B_EN_A_PIN, HIGH);
      digitalWrite(MOT_B_EN_B_PIN, LOW);
   }else{//backward
     digitalWrite(MOT_B_EN_A_PIN, LOW);
     digitalWrite(MOT_B_EN_B_PIN, HIGH);
   } 
   motor_speed = motor_speed > 0 ? motor_speed * 255 : (-motor_speed * 255) ;
   analogWrite(MOT_B_PWM_PIN, motor_speed);
}


void setMotorASpeedMC33886(float motor_speed){
  if(motor_speed < 0.005  && motor_speed > -0.005){ //brake
      digitalWrite(MOT_A_EN_B_PIN, HIGH);
      analogWrite(MOT_A_PWM_PIN, 255);
  }else if(motor_speed > 0){ //foward
     motor_speed =  255.0 * (motor_speed) ;
     unsigned char speed_char = motor_speed ;
     digitalWrite(MOT_A_EN_B_PIN, LOW);
     analogWrite(MOT_A_PWM_PIN, speed_char);
   }else{//backward
     motor_speed = 255.0 * (1 + motor_speed) ;
     unsigned char speed_char = motor_speed ;
     digitalWrite(MOT_A_EN_B_PIN, HIGH);
     analogWrite(MOT_A_PWM_PIN, speed_char);
   } 
}

void setMotorBSpeedMC33886(float motor_speed){
  if(motor_speed < 0.01  && motor_speed > -0.01){ //brake
      digitalWrite(MOT_B_EN_B_PIN, HIGH);
      analogWrite(MOT_B_PWM_PIN, 255);
  }else if(motor_speed > 0.){ //foward
     motor_speed =  255.0 *  motor_speed ;
     unsigned char speed_char = motor_speed ;
     digitalWrite(MOT_B_EN_B_PIN, LOW);
     analogWrite(MOT_B_PWM_PIN,  motor_speed);
   }else{//backward
     motor_speed = 255.0 * (1+motor_speed) ;
     unsigned char speed_char = motor_speed ;
     digitalWrite(MOT_B_EN_B_PIN, HIGH);
     analogWrite(MOT_B_PWM_PIN, speed_char);
   } 
}


void setMotorASpeed(float motor_speed){
  #ifdef MC33886
  setMotorASpeedMC33886(motor_speed);
  #else
  setMotorASpeedVNH2SP30(motor_speed);
  #endif
}

void setMotorBSpeed(float motor_speed){
  #ifdef MC33886
  setMotorBSpeedMC33886(motor_speed);
  #else
  setMotorBSpeedVNH2SP30(motor_speed);
  #endif
}

void init_motors(){
    pinMode(MOT_A_PWM_PIN, OUTPUT);
    //pinMode(MOT_A_EN_A_PIN, OUTPUT);
    pinMode(MOT_A_EN_B_PIN, OUTPUT);

    pinMode(MOT_B_PWM_PIN, OUTPUT);
    //pinMode(MOT_B_EN_A_PIN, OUTPUT);
    pinMode(MOT_B_EN_B_PIN, OUTPUT);


    //starting with break on motors
    setMotorBSpeed(0.0);
    setMotorASpeed(0.0);  
}


#define WAIT_INIT_SAMPLES 300 //300 samples equals to 3 seconds to let PMU6150 init
#define SET_P 0.22
#define SET_I 0.018
#define SET_D 0.000

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
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


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
  if(state->integral > 100) state->integral = 100 ;
    if(state->integral < -100) state->integral = -100 ;

  float pid_output = state->P * state->error ; 
  pid_output += state->I * state->integral ;
  if(state->derivative){
    pid_output += state->D * state->derivative ;
  }
  state->init_derivative = 1 ;
/*
  Serial.print(state->error);
  Serial.print(" : ");
  Serial.print(state->integral);
  Serial.print(" : ");
  Serial.print(state->derivative);
  Serial.print(" ->  ");
  Serial.println(pid_output);
  */
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
    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    //configure motors
    init_motors();

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize(200); //200Hz update rate

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-44);
    mpu.setYGyroOffset(-11);
    mpu.setZGyroOffset(19);
    mpu.setXAccelOffset(-897);
    mpu.setYAccelOffset(1109);
    mpu.setZAccelOffset(1694);

    

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(1, dmpDataReady, RISING);
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

    TCCR0B = (TCCR0B & ~(0x07)) | 0x02; //alter PWM frequency for pin 5,6
    init_pid(&current_state, SET_P, SET_I, SET_D);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
unsigned int init_measurement = WAIT_INIT_SAMPLES ;
float bot_angle =-1.2 ;
float steer_angle = 0.0 ;
char command_buffer [32] ;
unsigned char command_index = 0 ;
float old_angle = 0.0 ;
void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    //Serial.println("dmp ready");
    // wait for MPU interrupt or extra packet(s) available
    if (!mpuInterrupt) {
      if(Serial.available() > 0){
        char c = Serial.read() ;
        if(c == '\n' || c == '\r'){
          //todo: process command
          if(command_index > 1){
          switch(command_buffer[0]){
            case 'P': current_state.P = atof(&command_buffer[1]); 
              break ;
            case 'I': current_state.I = atof(&command_buffer[1]); 
              break ;
            case 'D': current_state.D = atof(&command_buffer[1]); 
               break ;
            case 'A': bot_angle = atof(&command_buffer[1]); 
               break ;
            case 'S': steer_angle = atof(&command_buffer[1]); 
               break ;
          }
          }
          command_index = 0 ;
          Serial.println("OK");
        }else{
          command_buffer[command_index]  = c ;
          command_index ++ ;
          if(command_index >=32) command_index = 0 ;
        }
      }
    }

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
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        float deg_angle = ypr[1] * 180/M_PI ; //could reason in radians but this eases the debug
        if(deg_angle > 6.0  || deg_angle < -6.0){
          bot_ready = 0  ;
          init_measurement = WAIT_INIT_SAMPLES;
          setMotorASpeed(0.0);
          setMotorBSpeed(0.0);
        }
        Serial.println(deg_angle);
        if(bot_ready){
          float motor_drive = update_pid(&current_state, deg_angle, bot_angle);
          if(motor_drive > 1.0) motor_drive = 1.0 ;
          if(motor_drive < -1.0) motor_drive = -1.0 ;
          float left_drive = motor_drive *(1.0 - steer_angle);
          float right_drive = motor_drive *(1.0 + steer_angle);
          setMotorASpeed(left_drive);
          setMotorBSpeed(right_drive);
        }else{
          if((deg_angle > 0.0 && deg_angle < 2.0 || deg_angle < 0.0 && deg_angle > -2.0 ) && init_measurement == 0){
            bot_ready = 1 ;
          }
          if(init_measurement > 0) init_measurement -- ;
        }
        old_angle = deg_angle ;
       
    }
    digitalWrite(LED_PIN, LOW);

}
