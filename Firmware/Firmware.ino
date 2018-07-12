#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//-------------Motor controller pins--------------//
#define STBY A0
#define RIGHTM_DIRECT1 9
#define RIGHTM_DIRECT2 10
#define RIGHTM_PWM 6

#define LEFTM_DIRECT1 3
#define LEFTM_DIRECT2 4
#define LEFTM_PWM 5 

#define MAX_SPEED 200 

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define INTERRUPT_PIN 2  // interrupt pin for IMU

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
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector from MPU6050
float ypr_angles[3];    // [yaw, pitch, roll]   pitch/roll 0-360 angles from MPU6050 and yaw from HMC5883L 

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//pid constants and limits
float pid_p_gain = 5.0;               //Gain setting for the pitch P-controller
float pid_i_gain = 0.2;               //Gain setting for the pitch I-controller
float pid_d_gain = 10.0;               //Gain setting for the pitch D-controller
int pid_max = 200;                    //Maximum output of the PID-controller (+/-)

//ypr errors 
float pitch_correction, roll_correction;
float current_pitch_error, current_roll_error;
float previous_pitch_error, previous_roll_error; 
float pid_i_error_pitch_total, pid_i_error_roll_total; 

//
byte base_speed = 20, motor_speed;
float pitch_set_point, roll_set_point = 5;
bool start = false;

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  
  mpu6050_init();
  wait_for_stable_sensor_data();
}

void loop() {
  ypr_read();

  if(start == false && (ypr_angles[2]<1.0 && ypr_angles[2]>-1.0)){
    start = true;
  }

  if(start==true){
    pid_calculation();

    if(roll_correction > MAX_SPEED)  roll_correction = MAX_SPEED;
    if(roll_correction < -MAX_SPEED)  roll_correction = -MAX_SPEED;

    if(roll_correction >= 0){
      motor_speed = (byte)roll_correction + base_speed;
      left_forward(motor_speed);
      right_forward(motor_speed);
    }
    if(roll_correction < 0){
      motor_speed = (byte)roll_correction*(-1) + base_speed;
      left_backward(motor_speed);
      right_backward(motor_speed);
    }
  }
}



void wait_for_stable_sensor_data(){
  int start;
  for(start = 0; start < 1000; start++){
    ypr_read();
  }
  Serial.println("calib");
}

void pid_calculation(){
  //roll error calculation
  current_roll_error = roll_set_point - ypr_angles[2];
  pid_i_error_roll_total += current_roll_error * pid_i_gain;
  if(pid_i_error_roll_total > pid_max){
    pid_i_error_roll_total = pid_max;
  }else if(pid_i_error_roll_total < pid_max*(-1)){
    pid_i_error_roll_total = pid_max*(-1);
  }

  roll_correction = pid_i_error_roll_total + current_roll_error * pid_p_gain + pid_d_gain * (current_roll_error - previous_roll_error);
  if(roll_correction > pid_max){
    roll_correction = pid_max;
  }else if(roll_correction < pid_max*(-1)){
    roll_correction = pid_max*(-1);
  }
  
  previous_roll_error = current_roll_error;
}
