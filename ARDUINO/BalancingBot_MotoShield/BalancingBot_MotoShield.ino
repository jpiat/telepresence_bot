

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"



#include "MPU6050_6Axis_MotionApps20.h"

#define MAX_ENCODER_TICK_PER_SEC 1133
#define MIN_ENCOER_TICK_PER_SEC 1 //arbitrarly defined
#define MAX_ENCODER_TICK_PER_10MS (MAX_ENCODER_TICK_PER_SEC/100)


//define for motor control pins
#define MOT_A_PWM 3
#define MOT_A_DIR 12
#define MOT_A_BRAKE 9

#define MOT_B_PWM 11
#define MOT_B_DIR 13
#define MOT_B_BRAKE 8

#define MOT_A_ENCA 4
#define MOT_A_ENCB 5

#define MOT_B_ENCA 6
#define MOT_B_ENCB 7

#define BT_PIN 10

#define FALL_ANGLE 10.0


void setMotorASpeed(float motor_speed){
   if(motor_speed < 0.005  && motor_speed > -0.005){ //brake
      digitalWrite(MOT_A_BRAKE, HIGH);
  }else if(motor_speed > 0){ //foward
      digitalWrite(MOT_A_BRAKE, LOW);
      digitalWrite(MOT_A_DIR, HIGH);
   }else{//backward
     digitalWrite(MOT_A_BRAKE, LOW);
     digitalWrite(MOT_A_DIR, LOW);
   } 
   motor_speed = motor_speed > 0 ? motor_speed * 255 : (-motor_speed * 255) ;
   analogWrite(MOT_A_PWM, motor_speed);
}

void setMotorBSpeed(float motor_speed){
   if(motor_speed < 0.005  && motor_speed > -0.005){ //brake
      digitalWrite(MOT_B_BRAKE, HIGH);
  }else if(motor_speed > 0){ //foward
      digitalWrite(MOT_B_BRAKE, LOW);
      digitalWrite(MOT_B_DIR, LOW);
   }else{//backward
     digitalWrite(MOT_B_BRAKE, LOW);
     digitalWrite(MOT_B_DIR, HIGH);
   } 
   motor_speed = motor_speed > 0 ? motor_speed * 255 : (-motor_speed * 255) ;
   analogWrite(MOT_B_PWM, motor_speed);
}


void pinInterruptSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}


unsigned char encoder_a_state = 0;
unsigned char encoder_b_state = 0;
int encoder_a_count = 0;
int encoder_b_count = 0;

unsigned char old_pin_state = 0;
//encoder interrupt
//encoder input are on D4 to D7 ...
ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
 {
    unsigned char new_pin_state = (PIND >> 4) & 0x0F ;
    unsigned char changes = (new_pin_state ^ old_pin_state) ;
    unsigned char b_change = (changes >> 2);
    unsigned char a_change = (changes & 0x03);
    if(a_change != 0){
        char a_edge = ((new_pin_state&0x01) < (old_pin_state&0x01)) ? -1: (a_change & 0x01);
        char b_edge = ((new_pin_state&0x02) < (old_pin_state&0x02)) ? -1: (a_change & 0x02);
      switch(encoder_a_state){
       case 0 :
          if(a_edge > 0) encoder_a_state ++ ;
          if(b_edge > 0) encoder_a_state = 3 ;
          break ;
       case 1 :
          if(b_edge > 0){
            encoder_a_state ++ ;
            encoder_a_count ++ ;
          }else if(a_edge < 0){
            encoder_a_state -- ;
            encoder_a_count -- ;
          }
          break ;
       case 2 :
          if(a_edge < 0) encoder_a_state ++ ;
          if(b_edge < 0) encoder_a_state -- ;
          break ;
       case 3 :
          if(b_edge < 0) encoder_a_state = 0 ;
          if(a_edge > 0) encoder_a_state -- ;
          break ;
       default :
        encoder_a_state = 0 ;
      }
      
    }else if(b_change != 0){
        char a_edge = ((new_pin_state&0x04) < (old_pin_state&0x04)) ? -1: (b_change & 0x01);
        char b_edge = ((new_pin_state&0x08) < (old_pin_state&0x08)) ? -1: (b_change & 0x02);
         switch(encoder_b_state){
         case 0 :
          if(a_edge > 0) encoder_b_state ++ ;
          if(b_edge > 0) encoder_b_state = 3 ;
          break ;
       case 1 :
          if(b_edge > 0){
            encoder_b_state ++ ;
            encoder_b_count ++ ;
          }else if(a_edge < 0){
            encoder_b_state -- ;
            encoder_b_count -- ;
          }
          break ;
       case 2 :
          if(a_edge < 0) encoder_b_state ++ ;
          if(b_edge < 0) encoder_b_state -- ;
          break ;
       case 3 :
          if(b_edge < 0) encoder_b_state = 0 ;
          if(a_edge > 0) encoder_b_state -- ;
          break ;
       default :
        encoder_a_state = 0 ;
      }
    }
    old_pin_state = new_pin_state;
 } 

void init_motors(){
    pinMode(MOT_A_PWM, OUTPUT);
    pinMode(MOT_A_DIR, OUTPUT);
    pinMode(MOT_A_BRAKE, OUTPUT);

    pinMode(MOT_B_PWM, OUTPUT);
    pinMode(MOT_B_DIR, OUTPUT);
    pinMode(MOT_B_BRAKE, OUTPUT);

    //starting with break on motors
    setMotorBSpeed(0.0);
    setMotorASpeed(0.0);  
    digitalWrite(MOT_A_ENCA,HIGH);
    digitalWrite(MOT_A_ENCB,HIGH);
    digitalWrite(MOT_B_ENCA,HIGH);
    digitalWrite(MOT_B_ENCB,HIGH);
    pinInterruptSetup(MOT_A_ENCA);
    pinInterruptSetup(MOT_B_ENCA);
    pinInterruptSetup(MOT_A_ENCB);
    pinInterruptSetup(MOT_B_ENCB);
}


#define WAIT_INIT_SAMPLES 300 //300 samples equals to 3 seconds to let PMU6150 init
#define SET_P 0.15
#define SET_I 0.018
#define SET_D 0.01

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu(0x69) ; // <-- use for AD0 high

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




struct pid_state angle_state ;
struct pid_state speed_state ;
struct pid_state motor_a_pid ;
struct pid_state motor_b_pid ;


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
    Serial.begin(57600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    pinMode(BT_PIN, OUTPUT);
    digitalWrite(BT_PIN, HIGH);
    //configure motors
    init_motors();

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize(200); //200Hz update rate

    // supply your own gyro offsets here, scaled for min sensitivity

    //-2611  -363  1063  12  31  -11

    mpu.setXGyroOffset(12);
    mpu.setYGyroOffset(31);
    mpu.setZGyroOffset(-11);
    mpu.setXAccelOffset(-2611);
    mpu.setYAccelOffset(-363);
    mpu.setZAccelOffset(1063);

    

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
    } /*else {
      //if configuration of IMU failed, go to blink of death
        while(1){
          digitalWrite(LED_PIN, HIGH);
          delay(200);
          digitalWrite(LED_PIN, LOW);
          delay(200);
        }
    }*/

    TCCR2B = (TCCR2B & ~(0x07)) | 0x03; //alter PWM frequency for pin 3, 11
    init_pid(&angle_state, SET_P, SET_I, SET_D);
    init_pid(&speed_state, 0.005, 0, 0);
    /*init_pid(&motor_a_pid, , , );
    init_pid(&motor_b_pid, , , );*/
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
unsigned int init_measurement = WAIT_INIT_SAMPLES ;
float balanced_angle = 0.0 ;
float bot_angle = 0.0 ;
float steer_angle = 0.0 ;
float set_speed = 0.0 ;
char command_buffer [32] ;
unsigned char command_index = 0 ;
float old_angle = 0.0 ;
unsigned char loop_modulo = 0 ;

void process_command(){
   if(Serial.available() > 0){
        char c = Serial.read() ;
        if(c == '\n' || c == '\r'){
          //todo: process command
          if(command_index > 1){
          switch(command_buffer[0]){
            case 'P': 
               angle_state.P = atof(&command_buffer[1]); 
               Serial.println("OK");
              break ;
            case 'I': 
                angle_state.I = atof(&command_buffer[1]);   
                Serial.println("OK");
              break ;
            case 'D': 
                angle_state.D = atof(&command_buffer[1]); 
                Serial.println("OK");
               break ;
            case 'V': 
                set_speed = atof(&command_buffer[1]); 
                Serial.println("OK");
                break ;
            case 'S': 
                steer_angle = atof(&command_buffer[1]); 
                Serial.println("OK");
                break ;
            case 'G':
               Serial.println(old_angle);
               Serial.println("OK");
               break ;
            default :
               Serial.println("NOK");
          }
          }
          command_index = 0 ;
        }else{
          command_buffer[command_index]  = c ;
          command_index ++ ;
          if(command_index >=32) command_index = 0 ;
        }
      }
}


void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady){
      Serial.println("Problem getting DMP ");
      delay(1000);
      return;
    }
    //Serial.println("Running ! ");
    //Serial.println("dmp ready");
    // wait for MPU interrupt or extra packet(s) available
    if (!mpuInterrupt) {
      process_command();
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if(fifoCount == 0) return  ; // no data available

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
        if(deg_angle > FALL_ANGLE  || deg_angle < -FALL_ANGLE){
          bot_ready = 0  ;
          init_measurement = WAIT_INIT_SAMPLES;
          setMotorASpeed(0.0);
          setMotorBSpeed(0.0);
        }
        if(bot_ready){
          float motor_drive = update_pid(&angle_state, deg_angle, bot_angle);
          if(motor_drive > 1.0) motor_drive = 1.0 ;
          if(motor_drive < -1.0) motor_drive = -1.0 ;
          float left_drive = motor_drive *(1.0 - steer_angle);
          float right_drive = motor_drive *(1.0 + steer_angle);
          setMotorASpeed(left_drive);
          setMotorBSpeed(right_drive);
        }else{
          if((deg_angle > 0.0 && deg_angle < 5.0 || deg_angle < 0.0 && deg_angle > -5.0 ) && init_measurement == 0){
            bot_ready = 1 ;
            Serial.println("READY");
          }
          if(init_measurement > 0) init_measurement -- ;
        }
        //Serial.println(deg_angle);
        old_angle = deg_angle ;
        loop_modulo ++ ;
        //do at lower rate
        /*if(loop_modulo >= 10){
          int linear_speed ;
          loop_modulo = 0 ;
           linear_speed = (encoder_a_count - encoder_b_count)/2;
           float angle_adjust = update_pid(&speed_state, linear_speed, set_speed);
           bot_angle += angle_adjust ;
           encoder_a_count = 0 ;
           encoder_b_count = 0 ;



           
        }*/

    }

}
