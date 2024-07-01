#include "prodigimover_small_odom.h"

ProdigimoverOdom::ProdigimoverOdom():
    linear_velocity_x_(0),
    linear_velocity_y_(0),
    linear_acceleration_x_(0),
    linear_acceleration_y_(0),
    linear_acceleration_z_(0),
    angular_velocity_x_(0),
    angular_velocity_y_(0),
    angular_velocity_z_(0),
    magnetic_field_x_(0),
    magnetic_field_y_(0),
    magnetic_field_z_(0),
    ax_(0),
    ay_(0),
    az_(0),
    gx_(0),
    gy_(0),
    gz_(0),
    mx_(0),
    my_(0),
    mz_(0),
    last_loop_time_(0),
    last_vel_time_(0),
    last_imu_time_(0),
    last_mag_time_(0),
    vel_dt_(0),
    x_pos_(0),
    y_pos_(0),
    heading_(0),
    imu_dt_(0),
    mag_dt_(0)
{
    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("raw_odom", 50);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_raw", 50);
    mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("/imu/mag", 50);
    velocity_subscriber_ = nh_.subscribe("cmd_vel", 50, &ProdigimoverOdom::velCallback, this);
    imu_subscriber_ = nh_.subscribe("raw_imu", 50, &ProdigimoverOdom::IMUCallback, this);
    mag_subscriber_ = nh_.subscribe("raw_mag", 50, &ProdigimoverOdom::MagCallback, this);
}

void ProdigimoverOdom::velCallback(const geometry_msgs::Twist& vel)
{
    ros::Time current_time = ros::Time::now();

    linear_velocity_x_ = vel.linear.x;
    linear_velocity_y_ = vel.linear.y;

    vel_dt_ = (current_time - last_vel_time_).toSec();
    last_vel_time_ = current_time;
}

void ProdigimoverOdom::IMUCallback(const sensor_msgs::Imu& imu){
  //callback every time the robot's angular velocity is received
  ros::Time current_time = ros::Time::now();
  
  //imu.header.frame_id ="imu_link";
  
  ax_ = imu.linear_acceleration.x;
  ay_ = imu.linear_acceleration.y;
  az_ = imu.linear_acceleration.z;
  
  gx_ = imu.angular_velocity.x;
  gy_ = imu.angular_velocity.y;
  gz_ = imu.angular_velocity.z;
  //this block is to filter out imu noise
  if(imu.angular_velocity.z > -0.03 && imu.angular_velocity.z < 0.03)
  {
    az_ = 0.00;
  }
  else
  {
    az_ = imu.angular_velocity.z;
  }
  
  imu_pub_.publish(imu);

  imu_dt_ = (current_time - last_imu_time_).toSec();
  last_imu_time_ = current_time;


    double x_pos = 0.0;
    double y_pos = 0.0;
    double theta = 0.0;



    ros::spinOnce();
    

    //linear velocity is the linear velocity published from the ESP32 board in x axis
    //double linear_velocity_x;

    //linear velocity is the linear velocity published from the ESP32 board in y axis
    //double linear_velocity_y;

    //angular velocity is the rotation in Z from imu_filter_madgwick's output
    double angular_velocity_z = az_;

    //calculate angular displacement  θ = ω * t
    double delta_heading = angular_velocity_z_ * imu_dt_; //radians
    double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_; //m
    double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_; //m

    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle (geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);)
    odom_quat.setRPY(0,0,heading_);

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "robot_footprint";

    //robot's position in x,y, and z
    odom_trans.transform.translation.x = x_pos_;
    odom_trans.transform.translation.y = y_pos_;
    odom_trans.transform.translation.z = 0.0;

    //robot's heading in quaternion
    odom_trans.transform.rotation.x = odom_quat.x();
    odom_trans.transform.rotation.y = odom_quat.y();
    odom_trans.transform.rotation.z = odom_quat.z();
    odom_trans.transform.rotation.w = odom_quat.w();
    odom_trans.header.stamp = current_time;

    //publish robot's tf using odom_trans object
    //odom_broadcaster_.sendTransform(odom_trans);

    odom_cal.header.stamp = current_time;
    //odom.header.frame_id = "odom";
    //odom.child_frame_id = "robot_footprint";

    //robot's position in x,y, and z
    odom_cal.pose.pose.position.x = x_pos_;
    odom_cal.pose.pose.position.y = y_pos_;
    odom_cal.pose.pose.position.z = 0.0;

    //robot's heading in quaternion
    odom_cal.pose.pose.orientation.x = odom_quat.x();
    odom_cal.pose.pose.orientation.y = odom_quat.y();
    odom_cal.pose.pose.orientation.z = odom_quat.z();
    odom_cal.pose.pose.orientation.w = odom_quat.w();
    odom_cal.pose.covariance[0] = 0.001;
    odom_cal.pose.covariance[7] = 0.001;
    odom_cal.pose.covariance[35] = 0.001;

    //linear speed from encoders
    odom_cal.twist.twist.linear.x = linear_velocity_x_;
    odom_cal.twist.twist.linear.y = linear_velocity_y_;
    odom_cal.twist.twist.linear.z = 0.0;

    odom_cal.twist.twist.angular.x = 0.0;
    odom_cal.twist.twist.angular.y = 0.0;

    //angular speed from IMU
    odom_cal.twist.twist.angular.z = angular_velocity_z_;
    odom_cal.twist.covariance[0] = 0.0001;
    odom_cal.twist.covariance[7] = 0.0001;
    odom_cal.twist.covariance[35] = 0.0001;

    odom_publisher_.publish(odom_cal);
}

void ProdigimoverOdom::MagCallback(const sensor_msgs::MagneticField& mag)
{
    ros::Time current_time = ros::Time::now();

    mx_ = mag.magnetic_field.x;
    my_ = mag.magnetic_field.y;
    mz_ = mag.magnetic_field.z;
    
    mag_pub_.publish(mag);

    mag_dt_ = (current_time - last_vel_time_).toSec();
    last_vel_time_ = current_time;
}

