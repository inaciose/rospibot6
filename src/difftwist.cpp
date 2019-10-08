#include <ros/ros.h>

// i2c
#include "I2Cdev.h"
#include "globals.h"
#include "mcu.h"
#include "difftwist_globals.h"
#include "odometry_globals.h"
#include "commands_def.h"
#include "pid_def.h"
#include "pid_globals.h"

double leftMotorSpeedRequest;
double rightMotorSpeedRequest;

int leftMotorSpeedRequestPulses;
int rightMotorSpeedRequestPulses;

int maxSpeedPulses = 45; //TODO
double cmd_vel_k = 1.0;

void difftwist_ros_params_setup(ros::NodeHandle pn) {
    pn.param<double>("cmdvel_k", cmd_vel_k, 1.0);
    std::cout << "cmdvel_k: " << cmd_vel_k << std::endl;
}

void difftwist_compute(bool pidComputeEnabled) {
        // calculate motors speed
        leftMotorSpeedRequest = 1.0 * cmd_vel_x - cmd_vel_z * wheel_sep / 2;
        rightMotorSpeedRequest = 1.0 * cmd_vel_x + cmd_vel_z * wheel_sep / 2;

        //leftMotorSpeedRequest = (cmd_vel_x/wheel_per) - ((cmd_vel_z*wheel_sep)/wheel_per);
        //rightMotorSpeedRequest = (cmd_vel_x/wheel_per) + ((cmd_vel_z*wheel_sep)/wheel_per);

        // convert to pulses  //TODO better
        rightMotorSpeedRequestPulses = (rightMotorSpeedRequest * pulses_per_m / (1.0 / PID_SAMPLE_TIME)) * cmd_vel_k;
        leftMotorSpeedRequestPulses = (leftMotorSpeedRequest * pulses_per_m / (1.0 /PID_SAMPLE_TIME)) * cmd_vel_k;

        if(leftMotorSpeedRequestPulses < 0) {
            if(abs(leftMotorSpeedRequestPulses) > maxSpeedPulses) leftMotorSpeedRequestPulses = -maxSpeedPulses;
        } else {
           if(leftMotorSpeedRequestPulses > maxSpeedPulses) leftMotorSpeedRequestPulses = maxSpeedPulses;
        }

        if(rightMotorSpeedRequestPulses < 0) {
            if(abs(rightMotorSpeedRequestPulses) > maxSpeedPulses) rightMotorSpeedRequestPulses = -maxSpeedPulses;
        } else {
           if(rightMotorSpeedRequestPulses > maxSpeedPulses) rightMotorSpeedRequestPulses = maxSpeedPulses;
        }

        //ROS_INFO("Setpoint %f %f %d %d", leftMotorSpeedRequest, rightMotorSpeedRequest, leftMotorSpeedRequestPulses, rightMotorSpeedRequestPulses);

        if(pidComputeEnabled) {
            // set left motors direction and speed
            if(leftMotorSpeedRequestPulses > 0) {
                leftSpeedPidSetPoint = round(leftMotorSpeedRequestPulses);
            } else if(leftMotorSpeedRequest < 0) {
                leftSpeedPidSetPoint = round(abs(leftMotorSpeedRequestPulses));
            } else {
              leftSpeedPidSetPoint = 0;
            }

            // set right motors direction and speed
            if(rightMotorSpeedRequest > 0) {
                rightSpeedPidSetPoint = round(rightMotorSpeedRequestPulses);
            } else if(rightMotorSpeedRequest < 0) {
                rightSpeedPidSetPoint = round(abs(rightMotorSpeedRequestPulses));
            } else {
                rightSpeedPidSetPoint = 0;
            }
        } else {
            uint8_t* dataptr = (uint8_t*)tmp_data;
            dataptr++;
            tmp_data[1].i = leftMotorSpeedRequestPulses;
            tmp_data[2].i = rightMotorSpeedRequestPulses;
            I2Cdev::writeBytes(I2CADDR_STM32, CMD_MCU_SET_VEL, 11, dataptr);
        }

}
