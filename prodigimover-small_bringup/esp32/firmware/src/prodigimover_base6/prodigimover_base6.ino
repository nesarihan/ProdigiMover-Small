#define ROSSERIAL_ARDUINO_TCP

#include "WiFi.h"
//#include <Arduino.h>

const char* ssid = "AirTies_Air5442";
const char* password = "77P3I95eHB";

IPAddress server(10, 1, 9, 240);
uint16_t serverPort = 11411;


#include <ESP32Encoder.h>
#include <ros.h>
#include <ros/time.h>
#include <Wire.h>
#include <prodigimover_msgs/Velocities.h> //header file for publishing velocities for odom
#include <geometry_msgs/Twist.h> //header file for cmd_subscribing to "cmd_vel"
#include <prodigimover_msgs/PID.h> //header file for pid server
#include <sensor_msgs/Imu.h> //header files for imu
#include <sensor_msgs/MagneticField.h> //header files for magnetometer

#include "prodigimover_base_config.h"
#include "Motor.h"
#include "Kinematics.h"
#include "PID.h"
#include "imu.h"


#define IMU_PUBLISH_RATE 15 //hz
#define VEL_PUBLISH_RATE 30//hz
#define COMMAND_RATE 30 //hz
#define DEBUG_RATE 5

//left side motors
Motor motor1(MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B); // front
Motor motor3(MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B); // rear
// right side motors
Motor motor2(MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); // front
Motor motor4(MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B); // frear


int Motor::counts_per_rev_ = COUNTS_PER_REV; //COUNTS_PER_REV = 0 if no encoder
double g_req_angular_vel_z = 0;
double g_req_linear_vel_x = 0;
unsigned long g_prev_command_time = 0;
unsigned long g_prev_control_time = 0;
unsigned long g_publish_vel_time = 0;
unsigned long g_prev_imu_time = 0;
unsigned long g_prev_debug_time = 0;
bool imu_is_initialized;

bool g_is_first = true;

char g_buffer[50];

static IRAM_ATTR void enc1_cb(void*arg){
  ESP32Encoder* enc1 = (ESP32Encoder*) arg;
  //Serial.printf("enc1 cb: count: %d\n", enc1->getCount());
}

static IRAM_ATTR void enc2_cb(void*arg){
  ESP32Encoder* enc2 = (ESP32Encoder*) arg;
  //Serial.printf("enc1 cb: count: %d\n", enc1->getCount());
}

static IRAM_ATTR void enc3_cb(void*arg){
  ESP32Encoder* enc3 = (ESP32Encoder*) arg;
  //Serial.printf("enc1 cb: count: %d\n", enc1->getCount());
}

static IRAM_ATTR void enc4_cb(void*arg){
  ESP32Encoder* enc4 = (ESP32Encoder*) arg;
  //Serial.printf("enc1 cb: count: %d\n", enc1->getCount());
}

//left side encoders
ESP32Encoder encoder1(true, enc1_cb); //front
ESP32Encoder encoder3(true, enc3_cb); //rear

//right side encoders
ESP32Encoder encoder2(true, enc2_cb); //front
ESP32Encoder encoder4(true, enc4_cb); //rear

Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE, PWM_BITS);

PID motor1_pid(-255, 255, K_P, K_I, K_D);
PID motor2_pid(-255, 255, K_P, K_I, K_D);
PID motor3_pid(-255, 255, K_P, K_I, K_D);
PID motor4_pid(-255, 255, K_P, K_I, K_D);

//callback function prototypes
void commandCallback(const geometry_msgs::Twist& cmd_msg);
void PIDCallback(const prodigimover_msgs::PID& pid);

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<prodigimover_msgs::PID> pid_sub("pid", PIDCallback);

sensor_msgs::Imu raw_imu_msg;
sensor_msgs::MagneticField raw_mag_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
ros::Publisher raw_mag_pub("raw_mag", &raw_mag_msg);

prodigimover_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

void setup() {
  setupWiFi();
  nh.initNode();
  nh.getHardware()->setConnection(server, serverPort);
  nh.subscribe(pid_sub);
  nh.subscribe(cmd_sub);
  nh.advertise(raw_vel_pub);
  nh.advertise(raw_imu_pub);
  nh.advertise(raw_mag_pub);
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder1.attachHalfQuad(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B);
  encoder2.attachHalfQuad(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B);
  encoder3.attachHalfQuad(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B);
  encoder4.attachHalfQuad(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B);
  encoder1.setCount(0);
  encoder2.setCount(0);
  encoder3.setCount(0);
  encoder4.setCount(0);
  //encoder1.setFilter(1023);
  //encoder2.setFilter(1023);
  //encoder3.setFilter(1023);
  //encoder4.setFilter(1023);
  while (!nh.connected())
  {
    nh.spinOnce();
  }
  nh.loginfo("PRODIGIMOVER_BASE CONNECTED");

  Wire.begin();
  delay(1);
}


void setupWiFi(){
  WiFi.begin(ssid, password);
  //while(WiFi.status() != WL_CONNECTED) {delay(500); Serial.print(".");}
  //Serial.print("SSID: ");
  //Serial.println(WiFi.SSID());
  //Serial.print("IP: ");
  //Serial.println(WiFi.localIP());
}


void PIDCallback(const prodigimover_msgs::PID& pid)
{
  //callback function every time PID constants are received from pid for tuning
  //this callback receives pid object where P,I, and D constants are stored
  motor1_pid.updateConstants(pid.p, pid.i, pid.d);
  motor2_pid.updateConstants(pid.p, pid.i, pid.d);
  motor3_pid.updateConstants(pid.p, pid.i, pid.d);
  motor4_pid.updateConstants(pid.p, pid.i, pid.d);
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
  //callback function every time linear and angular speed is received from 'cmd_vel' topic
  //this callback function receives cmd_msg object where linear and angular speed are stored
  g_req_linear_vel_x = cmd_msg.linear.x;
  g_req_angular_vel_z = cmd_msg.angular.z;

  g_prev_command_time = millis();
}

void loop() {
  //this block drives the robot based on defined rate
  if ((millis() - g_prev_control_time) >= (1000 / COMMAND_RATE))
  {
    moveBase();
    g_prev_control_time = millis();
  }

  //this block stops the motor when no command is received
  //if ((millis() - g_prev_command_time) >= 400)
  //{
  //  stopBase();
  //}

  //this block publishes velocity based on defined rate
  if ((millis() - g_publish_vel_time) >= (1000 / VEL_PUBLISH_RATE))
  {
    publishVelocities();
    g_publish_vel_time = millis();
  }
  
  //this block publishes the IMU data based on defined rate
  if ((millis() - g_prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
  {
     //sanity check if the IMU exits
    if (g_is_first)
    {
      //checkIMU();
    //}
    //else
    //{
      //publish the IMU data
      publishIMU();
      publishMag();
    }
    g_prev_imu_time = millis();
  }

  //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
  if(DEBUG)
  {
    if ((millis() - g_prev_debug_time) >= (1000 / DEBUG_RATE))
    {
      printDebug();
      g_prev_debug_time = millis();
    }
  }
  //call all the callbacks waiting to be called
  nh.spinOnce();
  
}

void moveBase()
{
  Kinematics::output req_rpm;
  //get the required rpm for each motor based on required velocities
  req_rpm = kinematics.getRPM(g_req_linear_vel_x, 0.0, g_req_angular_vel_z);

  //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
  //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
  motor1.spin(motor1_pid.compute(constrain(req_rpm.motor1, -MAX_RPM, MAX_RPM), motor1.rpm));
  motor3.spin(motor3_pid.compute(constrain(req_rpm.motor3, -MAX_RPM, MAX_RPM), motor3.rpm));
  motor2.spin(motor2_pid.compute(constrain(req_rpm.motor2, -MAX_RPM, MAX_RPM), motor2.rpm));
  motor4.spin(motor4_pid.compute(constrain(req_rpm.motor4, -MAX_RPM, MAX_RPM), motor4.rpm));
}

void stopBase()
{
  g_req_linear_vel_x = 0.0;
  g_req_angular_vel_z = 0.0;
}

void publishVelocities()
{
  //update the current speed of each motor based on encoder's count
  motor1.updateSpeed(encoder1.getCount());
  motor2.updateSpeed(encoder2.getCount());
  motor3.updateSpeed(encoder3.getCount());
  motor4.updateSpeed(encoder4.getCount());

  Kinematics::velocities vel;
  vel = kinematics.getVelocities(motor1.rpm, motor2.rpm, motor3.rpm, motor4.rpm);

  //fill in the object
  raw_vel_msg.linear_x = vel.linear_x;
  raw_vel_msg.linear_y = 0.0;
  raw_vel_msg.angular_z = vel.angular_z;

  //publish raw_vel_msg object to ROS
  raw_vel_pub.publish(&raw_vel_msg);
}

void checkIMU()
{
  //this function checks if IMU is present
  imu_is_initialized = initIMU();
  if(imu_is_initialized=true){
    nh.loginfo("IMU Initialized");
  }
  else {
    nh.logfatal("IMU failed to initialize. Check your IMU connection.");
  }
  g_is_first = false;

}

void publishIMU()
{
    raw_imu_msg.header.stamp=nh.now();
    raw_imu_msg.header.frame_id="imu_link";
    //measure accelerometer
    raw_imu_msg.linear_acceleration = readAccelerometer();

    //measure gyroscope
    raw_imu_msg.angular_velocity = readGyroscope();

   

    //publish raw_imu_msg object to ROS
    raw_imu_pub.publish(&raw_imu_msg);
  
}
void publishMag()
{
    raw_mag_msg.header.stamp=nh.now();
    raw_mag_msg.header.frame_id="imu_link";
    
    //measure magnetometer
    raw_mag_msg.magnetic_field = readMagnetometer();

    //publish raw_mag_msg object to ROS
    raw_mag_pub.publish(&raw_mag_msg);
}

void printDebug()
{
  sprintf (g_buffer, "Encoder FrontLeft: %ld", encoder1.getCount());
  nh.loginfo(g_buffer);
  sprintf (g_buffer, "Encoder RearLeft: %ld", encoder3.getCount());
  nh.loginfo(g_buffer);
  sprintf (g_buffer, "Encoder FrontRight: %ld", encoder2.getCount());
  nh.loginfo(g_buffer);
 sprintf (g_buffer, "Encoder RearRight: %ld", encoder4.getCount());
  nh.loginfo(g_buffer);
}
