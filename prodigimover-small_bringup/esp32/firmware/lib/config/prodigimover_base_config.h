#define DEBUG 1

#ifndef PRODIGIMOVER_BASE_CONFIG_H
#define PRODIGIMOVER_BASE_CONFIG_H


//uncomment the motor driver you're using
//#define L298_DRIVER
#define BTS7960_DRIVER

#define USE_MPU9250_IMU

#define DEBUG 0

//#define K_P 0.6 // P constant
//#define K_I 0.3 // I constant
//#define K_D 0.5 // D constant

//define your robot' specs here
//#define MAX_RPM 330               // motor's maximum RPM
//#define COUNTS_PER_REV 1550       // wheel encoder's no of ticks per rev
//#define WHEEL_DIAMETER 0.33      // wheel's diameter in meters
//#define PWM_BITS 8                // PWM Resolution of the microcontroller
//#define LR_WHEELS_DIST 0.235  // distance between left and right wheels
//#define FR_WHEELS_DIST 0.30   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN
//#define MAX_STEERING_ANGLE 0.415  // max steering angle. This only applies to Ackermann steering

//=================BIGGER ROBOT SPEC (BTS7960)=============================
#define K_P 0.05  // P constant
#define K_I 0.9   // I constant
#define K_D 0.1   // D constant

// define your robot' specs here
#define MAX_RPM 150               // motor's maximum RPM
#define COUNTS_PER_REV 4480       // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.26       // wheel's diameter in meters
#define PWM_BITS 8                // PWM Resolution of the microcontroller
#define LR_WHEELS_DISTANCE 0.486  // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.326  // distance between front and back wheels. Ignore this if you're on 2WD/ACKERMANN
//================= END OF BIGGER ROBOT SPEC =============================

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  
    MOTOR3  MOTOR4  (4WD/MECANUM)  
         BACK
*/

/// ENCODER PINS
#define MOTOR1_ENCODER_A 19
#define MOTOR1_ENCODER_B 20

#define MOTOR2_ENCODER_A 18
#define MOTOR2_ENCODER_B 17 

#define MOTOR3_ENCODER_A 39
#define MOTOR3_ENCODER_B 40

#define MOTOR4_ENCODER_A 41
#define MOTOR4_ENCODER_B 42

//MOTOR PINS
#ifdef L298_DRIVER
  #define MOTOR_DRIVER L298

  #define MOTOR1_PWM 10
  #define MOTOR1_IN_A 11
  #define MOTOR1_IN_B 12

  #define MOTOR2_PWM 1
  #define MOTOR2_IN_A 2
  #define MOTOR2_IN_B 3

  #define MOTOR3_PWM 13
  #define MOTOR3_IN_A 14
  #define MOTOR3_IN_B 15

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A 5
  #define MOTOR4_IN_B 6

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif 

#ifdef BTS7960_DRIVER
  #define MOTOR_DRIVER BTS7960  

  #define MOTOR1_PWM 10 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_A 11
  #define MOTOR1_IN_B 12

  #define MOTOR2_PWM 13 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_A 14
  #define MOTOR2_IN_B 15

  #define MOTOR3_PWM 1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_IN_A 2 
  #define MOTOR3_IN_B 3

  #define MOTOR4_PWM 4 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_IN_A 5
  #define MOTOR4_IN_B 6

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif

#endif


