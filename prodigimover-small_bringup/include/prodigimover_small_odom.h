#ifndef PRODIGIMOVER_SMALL_ODOM_H
#define PRODIGIMOVER_SMALL_ODOM_H

#include <ros/ros.h>
#include <prodigimover_msgs/Velocities.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>

class ProdigimoverOdom
{
public:
    ProdigimoverOdom();
    void velCallback(const geometry_msgs::Twist& vel);
    void IMUCallback(const sensor_msgs::Imu& imu);
    void MagCallback(const sensor_msgs::MagneticField& mag);

private:
    ros::NodeHandle nh_;
    ros::Publisher odom_publisher_;
    ros::Publisher imu_pub_;
    ros::Publisher mag_pub_;
    ros::Subscriber velocity_subscriber_;
    ros::Subscriber imu_subscriber_;
    ros::Subscriber mag_subscriber_;
    //tf2_ros::TransformBroadcaster odom_broadcaster_;
    tf2::Quaternion odom_quat;
    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Odometry odom_cal;
    sensor_msgs::Imu Imu;
    sensor_msgs::MagneticField mag;


    float linear_velocity_x_;
    float linear_velocity_y_;
    float linear_acceleration_x_;
    float linear_acceleration_y_;
    float linear_acceleration_z_;
    float angular_velocity_x_;
    float angular_velocity_y_;
    float angular_velocity_z_;
    float magnetic_field_x_;
    float magnetic_field_y_;
    float magnetic_field_z_;
    float ax_;
    float ay_;
    float az_;
    float gx_;
    float gy_;
    float gz_;
    float mx_;
    float my_;
    float mz_;
    ros::Time last_loop_time_;
    ros::Time last_vel_time_;
    ros::Time last_imu_time_;
    ros::Time last_mag_time_;
    float x_pos_;
    float y_pos_;
    float heading_;
    float vel_dt_;
    float imu_dt_;
    float mag_dt_;

};

#endif
