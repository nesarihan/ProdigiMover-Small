#ifndef _IMU2_H_
#define _IMU2_H_

#include "I2Cbus.hpp"
#include "imu_config.h"

#include <Wire.h>
#include "geometry_msgs/Vector3.h"

bool initIMU()
{
    Wire.begin();
    int ret;
    
    accelerometer.begin();
    ret = accelerometer.begin();
    if(ret < 0)
        return 1;

    gyroscope.begin();
    ret = gyroscope.begin();
    if(ret < 0)
        return 1;
  
    magnetometer.begin();
    ret = magnetometer.begin();
    if(ret < 0)
        return 1;

    return 0;
}

geometry_msgs::Vector3 readAccelerometer()
{
    geometry_msgs::Vector3 accel;
    float ax, ay, az;
    
    ax = accelerometer.getAccelX_mss();
    ay = accelerometer.getAccelY_mss();
    az = accelerometer.getAccelZ_mss();

    accel.x = ax * (double) ACCEL_SCALE * G_TO_ACCEL;
    accel.y = ay * (double) ACCEL_SCALE * G_TO_ACCEL;
    accel.z = az * (double) ACCEL_SCALE * G_TO_ACCEL;

    return accel;
}

geometry_msgs::Vector3 readGyroscope()
{
    geometry_msgs::Vector3 gyro;
    float gx, gy, gz;

    gx = gyroscope.getGyroX_rads();
    gy = gyroscope.getGyroY_rads();
    gz = gyroscope.getGyroZ_rads();

    gyro.x = gx * (double) GYRO_SCALE * DEG_TO_RAD;
    gyro.y = gy * (double) GYRO_SCALE * DEG_TO_RAD;
    gyro.z = gz * (double) GYRO_SCALE * DEG_TO_RAD;

    return gyro;
}

geometry_msgs::Vector3 readMagnetometer()
{
    geometry_msgs::Vector3 mag;
    float hx, hy, hz;

    hx = magnetometer.getMagX_uT();
    hy = magnetometer.getMagY_uT();
    hz = magnetometer.getMagZ_uT();

    mag.x = hx * (double) MAG_SCALE * UTESLA_TO_TESLA;
    mag.y = hy * (double) MAG_SCALE * UTESLA_TO_TESLA;
    mag.z = hz * (double) MAG_SCALE * UTESLA_TO_TESLA;

    return mag;
}

#endif
