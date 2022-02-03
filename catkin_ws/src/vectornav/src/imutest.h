/*
 * MIT License (MIT)
 *
 * Copyright (c) 2018 Dereck Wonnacott <dereck@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#include <iostream>
#include <cmath>

// No need to define PI twice if we already have it included...
//#define M_PI 3.14159265358979323846  /* M_PI */

// ROS Libraries
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/FluidPressure.h"
#include "std_srvs/Empty.h"
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vectornav/Ins.h>



ros::Publisher pubIMU, pubMag, pubGPS, pubOdom, pubTemp, pubPres, pubIns;
ros::ServiceServer resetOdomSrv;

XmlRpc::XmlRpcValue rpc_temp;

// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"
#include "vn/compositedata.h"
#include "vn/util.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

// Method declarations for future use.
void BinaryAsyncMessageReceived(void* userData, Packet& p, size_t index);

// Custom user data to pass to packet callback function
struct UserData {
    // the vectornav device identifier
    int device_family;
    // frame id used only for Odom header.frame_id
    std::string map_frame_id;
    // frame id used for header.frame_id of other messages and for Odom child_frame_id
    std::string frame_id;
    // Boolean to use ned or enu frame. Defaults to enu which is data format from sensor.
    bool tf_ned_to_enu;
    bool frame_based_enu;
    // Initial position after getting a GPS fix.
    vec3d initial_position;
    bool initial_position_set = false;

    //Unused covariances initialized to zero's
    boost::array<double, 9ul> linear_accel_covariance = { };
    boost::array<double, 9ul> angular_vel_covariance = { };
    boost::array<double, 9ul> orientation_covariance = { };

    // ROS header time stamp adjustments
    double average_time_difference{0};
    ros::Time ros_start_time;
    bool adjust_ros_timestamp{false};
};

// Basic loop so we can initilize our covariance parameters above
boost::array<double, 9ul> setCov(XmlRpc::XmlRpcValue rpc){
    // Output covariance vector
    boost::array<double, 9ul> output = { 0.0 };

    // Convert the RPC message to array
    ROS_ASSERT(rpc.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for(int i = 0; i < 9; i++){
        ROS_ASSERT(rpc[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        output[i] = (double)rpc[i];
    }
    return output;
}

// Reset initial position to current position
bool resetOdom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp, UserData *user_data)
{
    ROS_INFO("Reset Odometry");
    user_data->initial_position_set = false;
    return true;
}

// Assure that the serial port is set to async low latency in order to reduce delays and package pilup.
// These changes will stay effective until the device is unplugged
#if __linux__  || __CYGWIN__
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
bool optimize_serial_communication(std::string portName){
    int portFd = -1;

    portFd = ::open(
        portName.c_str(),
        O_RDWR | O_NOCTTY
    );

    if (portFd == -1)
    {
        ROS_WARN("Can't open port for optimization");
        return false;
    }

    ROS_INFO("Set port to ASYNCY_LOW_LATENCY");
    struct serial_struct serial;
    ioctl(portFd, TIOCGSERIAL, &serial);
    serial.flags |= ASYNC_LOW_LATENCY;
    ioctl(portFd, TIOCSSERIAL, &serial);
    ::close(portFd);
    return true;
}
#elif
bool optimize_serial_communication(str::string portName){
    return true;
}
#endif


//Helper function to create IMU message
void fill_imu_message(
    sensor_msgs::Imu &msgIMU, vn::sensors::CompositeData &cd, ros::Time &time, UserData *user_data)
{
    msgIMU.header.stamp = time;
    msgIMU.header.frame_id = user_data->frame_id;

    if (cd.hasQuaternion() && cd.hasAngularRate() && cd.hasAcceleration())
    {

        vec4f q = cd.quaternion();
        vec3f ar = cd.angularRate();
        vec3f al = cd.acceleration();

        if (cd.hasAttitudeUncertainty())
        {
            vec3f orientationStdDev = cd.attitudeUncertainty();
            msgIMU.orientation_covariance[0] = pow(orientationStdDev[2]*M_PI/180, 2); // Convert to radians Roll
            msgIMU.orientation_covariance[4] = pow(orientationStdDev[1]*M_PI/180, 2); // Convert to radians Pitch
            msgIMU.orientation_covariance[8] = pow(orientationStdDev[0]*M_PI/180, 2); // Convert to radians Yaw
        }

        //Quaternion message comes in as a Yaw (z) pitch (y) Roll (x) format
        if (user_data->tf_ned_to_enu)
        {
            // If we want the orientation to be based on the reference label on the imu
            tf2::Quaternion tf2_quat(q[0],q[1],q[2],q[3]);
            geometry_msgs::Quaternion quat_msg;

            if(user_data->frame_based_enu)
            {
                // Create a rotation from NED -> ENU
                tf2::Quaternion q_rotate;
                q_rotate.setRPY (M_PI, 0.0, M_PI/2);
                // Apply the NED to ENU rotation such that the coordinate frame matches
                tf2_quat = q_rotate*tf2_quat;
                quat_msg = tf2::toMsg(tf2_quat);

                // Since everything is in the normal frame, no flipping required
                msgIMU.angular_velocity.x = ar[0];
                msgIMU.angular_velocity.y = ar[1];
                msgIMU.angular_velocity.z = ar[2];

                msgIMU.linear_acceleration.x = al[0];
                msgIMU.linear_acceleration.y = al[1];
                msgIMU.linear_acceleration.z = al[2];
            }
            else
            {
                // put into ENU - swap X/Y, invert Z
                quat_msg.x = q[1];
                quat_msg.y = q[0];
                quat_msg.z = -q[2];
                quat_msg.w = q[3];

                // Flip x and y then invert z
                msgIMU.angular_velocity.x = ar[1];
                msgIMU.angular_velocity.y = ar[0];
                msgIMU.angular_velocity.z = -ar[2];
                // Flip x and y then invert z
                msgIMU.linear_acceleration.x = al[1];
                msgIMU.linear_acceleration.y = al[0];
                msgIMU.linear_acceleration.z = -al[2];

                if (cd.hasAttitudeUncertainty())
                {
                    vec3f orientationStdDev = cd.attitudeUncertainty();
                    msgIMU.orientation_covariance[0] = pow(orientationStdDev[1]*M_PI/180, 2); // Convert to radians pitch
                    msgIMU.orientation_covariance[4] = pow(orientationStdDev[0]*M_PI/180, 2); // Convert to radians Roll
                    msgIMU.orientation_covariance[8] = pow(orientationStdDev[2]*M_PI/180, 2); // Convert to radians Yaw
                }
            }

        msgIMU.orientation = quat_msg;
        }
        else
        {
            msgIMU.orientation.x = q[0];
            msgIMU.orientation.y = q[1];
            msgIMU.orientation.z = q[2];
            msgIMU.orientation.w = q[3];

            msgIMU.angular_velocity.x = ar[0];
            msgIMU.angular_velocity.y = ar[1];
            msgIMU.angular_velocity.z = ar[2];
            msgIMU.linear_acceleration.x = al[0];
            msgIMU.linear_acceleration.y = al[1];
            msgIMU.linear_acceleration.z = al[2];
        }
        // Covariances pulled from parameters
        msgIMU.angular_velocity_covariance = user_data->angular_vel_covariance;
        msgIMU.linear_acceleration_covariance = user_data->linear_accel_covariance;
    }
}

//Helper function to create magnetic field message
void fill_mag_message(
    sensor_msgs::MagneticField &msgMag, vn::sensors::CompositeData &cd, ros::Time &time, UserData *user_data)
{
    msgMag.header.stamp = time;
    msgMag.header.frame_id = user_data->frame_id;

    // Magnetic Field
    if (cd.hasMagnetic())
    {
        vec3f mag = cd.magnetic();
        msgMag.magnetic_field.x = mag[0];
        msgMag.magnetic_field.y = mag[1];
        msgMag.magnetic_field.z = mag[2];
    }
}

 //Helper function to create gps message
void fill_gps_message(
    sensor_msgs::NavSatFix &msgGPS, vn::sensors::CompositeData &cd, ros::Time &time, UserData *user_data)
{
    msgGPS.header.stamp = time;
    msgGPS.header.frame_id = user_data->frame_id;

    if(cd.hasPositionEstimatedLla())
    {
        vec3d lla = cd.positionEstimatedLla();

        msgGPS.latitude = lla[0];
        msgGPS.longitude = lla[1];
        msgGPS.altitude = lla[2];

        // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
        if(cd.hasPositionUncertaintyEstimated())
        {
            double posVariance = pow(cd.positionUncertaintyEstimated(), 2);
            msgGPS.position_covariance[0] = posVariance;    // East position variance
            msgGPS.position_covariance[4] = posVariance;    // North position vaciance
            msgGPS.position_covariance[8] = posVariance;    // Up position variance

            // mark gps fix as not available if the outputted standard deviation is 0
            if(cd.positionUncertaintyEstimated() != 0.0)
            {
                // Position available
                msgGPS.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
            } else {
                // position not detected
                msgGPS.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
            }

            // add the type of covariance to the gps message
            msgGPS.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        } else {
            msgGPS.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        }
    }
}

//Helper function to create odometry message
void fill_odom_message(
    nav_msgs::Odometry &msgOdom, vn::sensors::CompositeData &cd, ros::Time &time, UserData *user_data)
{
    msgOdom.header.stamp = time;
    msgOdom.child_frame_id = user_data->frame_id;
    msgOdom.header.frame_id = user_data->map_frame_id;

    if(cd.hasPositionEstimatedEcef())
    {
        // add position as earth fixed frame
        vec3d pos = cd.positionEstimatedEcef();

        if (!user_data->initial_position_set)
        {
            ROS_INFO("Set initial position to %f %f %f", pos[0], pos[1], pos[2]);
            user_data->initial_position_set = true;
            user_data->initial_position.x = pos[0];
            user_data->initial_position.y = pos[1];
            user_data->initial_position.z = pos[2];
        }

        msgOdom.pose.pose.position.x = pos[0] - user_data->initial_position[0];
        msgOdom.pose.pose.position.y = pos[1] - user_data->initial_position[1];
        msgOdom.pose.pose.position.z = pos[2] - user_data->initial_position[2];

        // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
        if(cd.hasPositionUncertaintyEstimated())
        {
            double posVariance = pow(cd.positionUncertaintyEstimated(), 2);
            msgOdom.pose.covariance[0] = posVariance;   // x-axis position variance
            msgOdom.pose.covariance[7] = posVariance;   // y-axis position vaciance
            msgOdom.pose.covariance[14] = posVariance;  // z-axis position variance
        }
    }

    if (cd.hasQuaternion())
    {
        vec4f q = cd.quaternion();

        if(!user_data->tf_ned_to_enu) {
            // output in NED frame
            msgOdom.pose.pose.orientation.x = q[0];
            msgOdom.pose.pose.orientation.y = q[1];
            msgOdom.pose.pose.orientation.z = q[2];
            msgOdom.pose.pose.orientation.w = q[3];
        } else if(user_data->tf_ned_to_enu && user_data->frame_based_enu) {
            // standard conversion from NED to ENU frame
            tf2::Quaternion tf2_quat(q[0],q[1],q[2],q[3]);
            // Create a rotation from NED -> ENU
            tf2::Quaternion q_rotate;
            q_rotate.setRPY (M_PI, 0.0, M_PI/2);
            // Apply the NED to ENU rotation such that the coordinate frame matches
            tf2_quat = q_rotate*tf2_quat;
            msgOdom.pose.pose.orientation = tf2::toMsg(tf2_quat);
        } else if(user_data->tf_ned_to_enu && !user_data->frame_based_enu) {
            // alternative method for conversion to ENU frame (leads to another result)
            // put into ENU - swap X/Y, invert Z
            msgOdom.pose.pose.orientation.x = q[1];
            msgOdom.pose.pose.orientation.y = q[0];
            msgOdom.pose.pose.orientation.z = -q[2];
            msgOdom.pose.pose.orientation.w = q[3];
        }

        // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
        if(cd.hasAttitudeUncertainty())
        {
            vec3f orientationStdDev = cd.attitudeUncertainty();
            // convert the standard deviation values from all three axis from degrees to radiant and calculate the variances from these (squared), which are assigned to the covariance matrix.
            if(!user_data->tf_ned_to_enu || user_data->frame_based_enu) {
                // standard assignment of variance values for NED frame and conversion to ENU frame by rotation
                msgOdom.pose.covariance[21] = pow(orientationStdDev[0] * M_PI / 180, 2);    // roll variance
                msgOdom.pose.covariance[28] = pow(orientationStdDev[1] * M_PI / 180, 2);    // pitch variance
                msgOdom.pose.covariance[35] = pow(orientationStdDev[2] * M_PI / 180, 2);    // yaw variance
            } else {
                // variance assignment for conversion by swapping and inverting (not frame_based_enu)

                // TODO not supported yet
            }
        }
    }

    // Add the velocity in the body frame (frame_id) to the message
    if (cd.hasVelocityEstimatedBody())
    {
        vec3f vel = cd.velocityEstimatedBody();

        if(!user_data->tf_ned_to_enu || user_data->frame_based_enu) {
            // standard assignment of values for NED frame and conversion to ENU frame by rotation
            msgOdom.twist.twist.linear.x = vel[0];
            msgOdom.twist.twist.linear.y = vel[1];
            msgOdom.twist.twist.linear.z = vel[2];
        } else {
            // value assignment for conversion by swapping and inverting (not frame_based_enu)
            // Flip x and y then invert z
            msgOdom.twist.twist.linear.x = vel[1];
            msgOdom.twist.twist.linear.y = vel[0];
            msgOdom.twist.twist.linear.z = -vel[2];
        }

        // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
        if(cd.hasVelocityUncertaintyEstimated())
        {
            double velVariance = pow(cd.velocityUncertaintyEstimated(), 2);
            msgOdom.twist.covariance[0] = velVariance;  // x-axis velocity variance
            msgOdom.twist.covariance[7] = velVariance;  // y-axis velocity vaciance
            msgOdom.twist.covariance[14] = velVariance; // z-axis velocity variance

            // set velocity variances to a high value if no data is available (this is the case at startup during INS is initializing)
            if(msgOdom.twist.twist.linear.x == 0 && msgOdom.twist.twist.linear.y == 0 && msgOdom.twist.twist.linear.z == 0 && msgOdom.twist.covariance[0] == 0 && msgOdom.twist.covariance[7] == 0 && msgOdom.twist.covariance[14] == 0){
                msgOdom.twist.covariance[0] = 200;
                msgOdom.twist.covariance[7] = 200;
                msgOdom.twist.covariance[15] = 200;
            }
        }
    }

    if (cd.hasAngularRate())
    {
        vec3f ar = cd.angularRate();

            if(!user_data->tf_ned_to_enu || user_data->frame_based_enu) {
            // standard assignment of values for NED frame and conversion to ENU frame by rotation
            msgOdom.twist.twist.angular.x = ar[0];
            msgOdom.twist.twist.angular.y = ar[1];
            msgOdom.twist.twist.angular.z = ar[2];
        } else {
            // value assignment for conversion by swapping and inverting (not frame_based_enu)
            // Flip x and y then invert z
            msgOdom.twist.twist.angular.x = ar[1];
            msgOdom.twist.twist.angular.y = ar[0];
            msgOdom.twist.twist.angular.z = -ar[2];
        }

        // add covariance matrix of the measured angular rate to odom message.
        // go through matrix rows
        for(int row = 0; row < 3; row++) {
            // go through matrix columns
            for(int col = 0; col < 3; col++) {
                // Target matrix has 6 rows and 6 columns, source matrix has 3 rows and 3 columns. The covariance values are put into the fields (3, 3) to (5, 5) of the destination matrix.
                msgOdom.twist.covariance[(row + 3) * 6 + (col + 3)] = user_data->angular_vel_covariance[row * 3 + col];
            }
        }
    }
}

 
//Helper function to create temperature message
void fill_temp_message(
    sensor_msgs::Temperature &msgTemp, vn::sensors::CompositeData &cd, ros::Time &time, UserData *user_data)
{
    msgTemp.header.stamp = time;
    msgTemp.header.frame_id = user_data->frame_id;
    if (cd.hasTemperature())
    {
        float temp = cd.temperature();
        msgTemp.temperature = temp;
    }
}

//Helper function to create pressure message
void fill_pres_message(
    sensor_msgs::FluidPressure &msgPres, vn::sensors::CompositeData &cd, ros::Time &time, UserData *user_data)
{
    msgPres.header.stamp = time;
    msgPres.header.frame_id = user_data->frame_id;
    if (cd.hasPressure())
    {
        float pres = cd.pressure();
        msgPres.fluid_pressure = pres;
    }
}

//Helper function to create ins message
void fill_ins_message(
    vectornav::Ins &msgINS, vn::sensors::CompositeData &cd, ros::Time &time, UserData *user_data)
{
    msgINS.header.stamp = time;
    msgINS.header.frame_id = user_data->frame_id;

    if (cd.hasInsStatus())
    {
        InsStatus insStatus = cd.insStatus();
        msgINS.insStatus = static_cast<uint16_t>(insStatus);
    }

    if (cd.hasTow()){
        msgINS.time = cd.tow();
    }

    if (cd.hasWeek()){
        msgINS.week = cd.week();
    }

    if (cd.hasTimeUtc()){
        TimeUtc utcTime = cd.timeUtc();
        char* utcTimeBytes = reinterpret_cast<char*>(&utcTime);
        //msgINS.utcTime bytes are in Little Endian Byte Order
        std::memcpy(&msgINS.utcTime, utcTimeBytes, 8);
    }

    if (cd.hasYawPitchRoll()) {
        vec3f rpy = cd.yawPitchRoll();
        msgINS.yaw = rpy[0];
        msgINS.pitch = rpy[1];
        msgINS.roll = rpy[2];
    }

    if (cd.hasPositionEstimatedLla()) {
        vec3d lla = cd.positionEstimatedLla();
        msgINS.latitude = lla[0];
        msgINS.longitude = lla[1];
        msgINS.altitude = lla[2];
    }

    if (cd.hasVelocityEstimatedNed()) {
        vec3f nedVel = cd.velocityEstimatedNed();
        msgINS.nedVelX = nedVel[0];
        msgINS.nedVelY = nedVel[1];
        msgINS.nedVelZ = nedVel[2];
    }

    if (cd.hasAttitudeUncertainty())
    {
        vec3f attUncertainty = cd.attitudeUncertainty();
        msgINS.attUncertainty[0] = attUncertainty[0];
        msgINS.attUncertainty[1] = attUncertainty[1];
        msgINS.attUncertainty[2] = attUncertainty[2];
    }

    if (cd.hasPositionUncertaintyEstimated()){
        msgINS.posUncertainty = cd.positionUncertaintyEstimated();
    }

    if (cd.hasVelocityUncertaintyEstimated()){
        msgINS.velUncertainty = cd.velocityUncertaintyEstimated();
    }
}


static ros::Time get_time_stamp(
    vn::sensors::CompositeData &cd, UserData *user_data, const ros::Time &ros_time) {
    if (!cd.hasTimeStartup() || !user_data->adjust_ros_timestamp) {
        return (ros_time); // don't adjust timestamp
    }
    const double sensor_time = cd.timeStartup() * 1e-9; // time in seconds
    if (user_data->average_time_difference == 0) { // first call
        user_data->ros_start_time = ros_time;
        user_data->average_time_difference = static_cast<double>(-sensor_time);
    }
    // difference between node startup and current ROS time
    const double ros_dt = (ros_time - user_data->ros_start_time).toSec();
    // difference between elapsed ROS time and time since sensor startup
    const double dt = ros_dt - sensor_time;
    // compute exponential moving average
    const double alpha = 0.001; // average over rougly 1000 samples
    user_data->average_time_difference =
        user_data->average_time_difference * (1.0 - alpha) + alpha * dt;

    // adjust sensor time by average difference to ROS time
    const ros::Time adj_time = user_data->ros_start_time + ros::Duration(
      user_data->average_time_difference + sensor_time);
    return (adj_time);
}

//
// Callback function to process data packet from sensor
//
void BinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
{
    // evaluate time first, to have it as close to the measurement time as possible
    const ros::Time ros_time = ros::Time::now();

    vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);
    UserData *user_data = static_cast<UserData*>(userData);
    ros::Time time = get_time_stamp(cd, user_data, ros_time);

    // IMU
    if (pubIMU.getNumSubscribers() > 0)
    {
        sensor_msgs::Imu msgIMU;
        fill_imu_message(msgIMU, cd, time, user_data);
        pubIMU.publish(msgIMU);
    }

    // Magnetic Field
    if (pubMag.getNumSubscribers() > 0)
    {
        sensor_msgs::MagneticField msgMag;
        fill_mag_message(msgMag, cd, time, user_data);
        pubMag.publish(msgMag);
    }

    // Temperature
    if (pubTemp.getNumSubscribers() > 0)
    {
        sensor_msgs::Temperature msgTemp;
        fill_temp_message(msgTemp, cd, time, user_data);
        pubTemp.publish(msgTemp);
    }

    // Barometer
    if (pubPres.getNumSubscribers() > 0)
    {
        sensor_msgs::FluidPressure msgPres;
        fill_pres_message(msgPres, cd, time, user_data);
        pubPres.publish(msgPres);
    }

    // GPS
    if (user_data->device_family != VnSensor::Family::VnSensor_Family_Vn100 && pubGPS.getNumSubscribers() > 0)
    {
        sensor_msgs::NavSatFix msgGPS;
        fill_gps_message(msgGPS, cd, time, user_data);
        pubGPS.publish(msgGPS);
    }

    // Odometry
    if (user_data->device_family != VnSensor::Family::VnSensor_Family_Vn100 && pubOdom.getNumSubscribers() > 0)
    {
        nav_msgs::Odometry msgOdom;
        fill_odom_message(msgOdom, cd, time, user_data);
        pubOdom.publish(msgOdom);
    }

    // INS
    if (user_data->device_family != VnSensor::Family::VnSensor_Family_Vn100 && pubIns.getNumSubscribers() > 0)
    {
        vectornav::Ins msgINS;
        fill_ins_message(msgINS, cd, time, user_data);
        pubIns.publish(msgINS);
    }
}
