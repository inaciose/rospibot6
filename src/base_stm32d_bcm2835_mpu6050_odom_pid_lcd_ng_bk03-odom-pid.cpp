#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>

#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <vector>
#include <iostream>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32MultiArray.h>

// lcd
#include "nokia5110.h"
#include "nokia5110_lcdmenu.h"

#include "system.h"
#include "commands.h"
#include "globals.h"

// i2c
#include "I2Cdev.h"

// allow to use other nodes
bool pidComputeEnabled = false;
bool odomPublishEnabled = false;
bool cmdVelComputeEnabled = false;

// LCD pins
#define LCD_PIN_BL    21
#define LCD_PIN_SCE   8
#define LCD_PIN_RESET 24
#define LCD_PIN_DC    23
#define LCD_PIN_SDIN  10
#define LCD_PIN_SCLK  11

#define LCD_TIMER 1
#define LCD_BACKLIGHT_TIMER 30

double lcdTimer;
double lcdBacklightTimer;
bool lcdBacklightStatus = false;

// push buttons
#define BTN_NUM 6

// push buttons labels
#define BTN_BUP    2
#define BTN_BDN    3
#define BTN_BLF    4
#define BTN_BRT    5
#define BTN_BNO    0
#define BTN_BOK    1

// push buttons pins
#define BTN_PIN_BUP    25
#define BTN_PIN_BDN    27
#define BTN_PIN_BLF    20
#define BTN_PIN_BRT    17
#define BTN_PIN_BNO    18
#define BTN_PIN_BOK    22

#define BTN_TIMER 0.005
#define BTN_ON_TIMER 0.6

bool btnStatus = false;
double btnTimer;
uint8_t btnState[BTN_NUM];

// lcd  menu
#define MENU_MAX 3
bool menuOn = false;
uint8_t menu = 0;
uint8_t menuselect[MENU_MAX+1][2] = {{0,3}, {0,3}, {0,2}, {0,1}}; // selected item , total_itens

// encoder
#include "odometry.h"
#include "odometry_def.h"
#include "odometry_globals.h"

// units are m, m/s, radian/s

// twist, pwm & pid variables
double cmd_vel_x, cmd_vel_z;
double leftMotorSpeedRequest,rightMotorSpeedRequest;
int leftMotorSpeedRequestPulses, rightMotorSpeedRequestPulses;
int maxSpeedPulses = 45; //TODO
double cmd_vel_k = 1.0;

//long encoderLeftSpeedPidPulses;
//long encoderRightSpeedPidPulses;

//double leftSpeedPidInput, rightSpeedPidInput;
//double leftSpeedPidSetPoint, rightSpeedPidSetPoint;

double pidTimer;
bool newPidPwm = false;
double leftPidPwm, rightPidPwm;

//PID constants
//#define PID_SETPOINT_MAX 100
//#define PID_OUT_TRH 0
//#define PID_OUT_MIN 0
//#define PID_OUT_MAX 32000
//#define PID_OUT_K 1
//#define PID_SAMPLE_TIME 0.025

#include "pid.h"
#include "pid_def.h"
#include "pid_globals.h"

/*
double pidOutTrh = PID_OUT_TRH;
double pidOutMin = PID_OUT_MIN;
double pidOutMax = PID_OUT_MAX;
double pidOutK = PID_OUT_K;
double pidSampleTime = PID_SAMPLE_TIME;
double pidSetPointMax =  PID_SETPOINT_MAX;

double kpLeft = 1;
double kiLeft = 0.0005;
double kdLeft = 0;

double kpRight = 1;
double kiRight = 0.0005;
double kdRight = 0;
*/

// control variables
bool newCmdVel = false;
double encoderPublisherTimer;
double tfPublisherTimer;

//
// MPU6050
//
#define DEFAULT_SAMPLE_RATE_HZ  10

#include "mpu6050_bcm2835.h"
#include "mpu6050_bcm2835_def.h"
#include "mpu6050_bcm2835_globals.h"

double imuPublisherTimer;

ros::Publisher imu_calib_pub;
ros::ServiceClient * clientptr;

int sample_rate;

ros::Publisher imu_pub;
ros::Publisher imu_euler_pub;
ros::Publisher mag_pub;

//
// MOTORS
//
//#define MAX_ENCODER_DIFF 5000

#include "mcu.h"
//#define I2CADDR_STM32 0x0b
//#define I2C_MAX_LEN 32
uint8_t i2c_buf[I2C_MAX_LEN];

// data trasfer variables
//union u_tag {
//    uint8_t b[4];
//    int32_t i;
//};

union u_tag pwm_data[8];
union u_tag enc_data[8];
union u_tag tmp_data[8];

bool newPwmData = false;

ros::Publisher left_encoder_pub;
ros::Publisher right_encoder_pub;

std_msgs::Int64 msgEnc;

//
// OTHER
//

union u_tag cmd_data[8];
uint16_t cmd = 0;
bool newCmd = false;

bool lidarMotorStatus = false;

bool newCmdMotion = false;
int cmdMotion[64];

// loop info for debug
bool displayLoopInfo = false;

void leftPwmCallback(std_msgs::Float32 msg) {
    //ROS_INFO("motorLeftCallback: %f", msg.data);
    pwm_data[1].i = (int32_t)msg.data;
    newPwmData = true;
}

void rightPwmCallback(std_msgs::Float32 msg) {
    //ROS_INFO("motorLeftCallback: %f", msg.data);
    pwm_data[2].i = (int32_t)msg.data;
    newPwmData = true;
}

void twistCallback(const geometry_msgs::Twist& msg) {
    //ROS_INFO("twistCallback: %f, %f", msg.linear.x, msg.angular.z);
    cmd_vel_x = msg.linear.x;
    cmd_vel_z = msg.angular.z;
    newCmdVel = true;
}

void cmdCallback(std_msgs::Int16 msg) {
    //ROS_INFO("cmdCallback: %f", msg.data);
    cmd = (int16_t)msg.data;
    newCmd = true;
}

void cmdMotionCallback(const std_msgs::Int32MultiArray::ConstPtr& array) {
    int i = 0;
    for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
    	cmdMotion[i] = *it;
    	i++;
    }
    //ROS_INFO("cmdMotionCallback: %d %d %d %d", cmdMotion[0], cmdMotion[1], cmdMotion[2], cmdMotion[3]);
    newCmdMotion = true;
}

void mySigintHandler(int sig){
    ROS_INFO("Shutting down base_node...");
    // stop lidar motor
    uint8_t* dataptr = (uint8_t*)tmp_data;
    dataptr++;

    I2Cdev::writeBytes(I2CADDR_STM32, CMD_MCU_LIDAR_MOTOR_OFF, 3, dataptr);

    // stop motors
    dataptr = (uint8_t*)pwm_data;
    dataptr++;

    pwm_data[1].i = 0;
    pwm_data[2].i = 0;
    I2Cdev::writeBytes(I2CADDR_STM32, CMD_MCU_SET_PWM, 11, dataptr);

    // reset mpu6050
    //mpu.reset();
    mpu6050_device_reset();

    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

// ================================================================
// ===                        MAIN LOOP                         ===
// ================================================================

void loop(ros::NodeHandle pn, ros::NodeHandle n, ros::Publisher odom_pub, tf::TransformBroadcaster odom_broadcaster) {

    //
    // PROCESS MOTORS
    //
    uint8_t i;
    uint8_t res1, res2, res3;

    ros::Time current_time;
    static ros::Time last_time;

    uint8_t* dataptr;

    // process new motion commands
    if(newCmdMotion) {
	dataptr = (uint8_t*)tmp_data;
        dataptr++;
	switch(cmdMotion[0]) {
	    case CMD_MCU_SET_MOTION:
                // left encoder marks for debug
                if(cmdMotion[1] >= 0) {
                    encoderLeftPulsesTarget = encoderLeftPulses + cmdMotion[3];
                    encoderLeftTargetDirection = 1;
                } else {
                    encoderLeftPulsesTarget = encoderLeftPulses - cmdMotion[3];
                    encoderLeftTargetDirection = -1;
                }
                // right encoder marks for debug
                if(cmdMotion[2] >= 0) {
                    encoderRightPulsesTarget = encoderRightPulses + cmdMotion[3];
                    encoderRightTargetDirection = 1;
                } else {
                    encoderRightPulsesTarget = encoderRightPulses - cmdMotion[3];
                    encoderRightTargetDirection = -1;
                }

                encoderLeftPulsesTargetStart = encoderLeftPulses;
                encoderRightPulsesTargetStart = encoderRightPulses;
                encoderLeftPulsesOnTarget = false;
                encoderRightPulsesOnTarget = false;
                encoderPulsesTargetEnabled = true;
	        tmp_data[1].i = cmdMotion[1];
                tmp_data[2].i = cmdMotion[2];
                tmp_data[3].i = cmdMotion[3];
        	I2Cdev::writeBytes(I2CADDR_STM32, CMD_MCU_SET_MOTION, 15, dataptr);
		break;
	}
        ROS_INFO("sendtomcu: %d %d %d %d", cmdMotion[0], cmdMotion[1], cmdMotion[2], cmdMotion[3]);
	newCmdMotion = false;
    }

    // publish encoders pulse count on timer
    if(ros::Time::now().toSec() > encoderPublisherTimer) {

	// read encoders position from mcu
	read_i2c_encoders();

	// debug
	check_encoders_position();

	// publish left encoder ros topic
	msgEnc.data = encoderLeftPulses;
	left_encoder_pub.publish(msgEnc);

	// publish right encoder ros topic
	msgEnc.data = encoderRightPulses;
	right_encoder_pub.publish(msgEnc);
	encoderPublisherTimer = ros::Time::now().toSec() + ENCODER_PUBLISHER_TIMER;

    }

    if(pidComputeEnabled && (ros::Time::now().toSec() > pidTimer)) {

	leftSpeedPidInput = abs(encoderLeftSpeedPidPulses);
	rightSpeedPidInput = abs(encoderRightSpeedPidPulses);
	leftPidPwm = computeLeftPID(leftSpeedPidInput);
	rightPidPwm = computeRightPID(rightSpeedPidInput);

	encoderLeftSpeedPidPulses = 0;
	encoderRightSpeedPidPulses = 0;

	//ROS_INFO("PID update %f %f %f %f %f %f %f %f", leftSpeedPidSetPoint - leftSpeedPidInput, rightSpeedPidSetPoint - rightSpeedPidInput, leftPidPwm, rightPidPwm,  leftSpeedPidInput, rightSpeedPidInput, leftSpeedPidSetPoint, rightSpeedPidSetPoint);

	// set left motor direction and velocity
	if(leftMotorSpeedRequest < 0) {
	    leftPidPwm = -leftPidPwm;
	}

	// set right motor direction and  velocity
	if(rightMotorSpeedRequest < 0) {
	    rightPidPwm = -rightPidPwm;
	}

	newPidPwm = true;

	pidTimer = ros::Time::now().toSec() + PID_SAMPLE_TIME;
    }

    // send pwm to slave
    //res2 = bcm2835_i2c_write((const char *)pwm_data, 8);
    uint8_t* dataptr1 = (uint8_t*)pwm_data;
    dataptr1++; 
    // need to skip one byte because the first byte is automaticaly sent as parameter
    // we use that byte to send the cmd to mcu
    res2 = 0;
    if(newPwmData) {
	res2 = I2Cdev::writeBytes(I2CADDR_STM32, CMD_MCU_SET_PWM, 11, dataptr1);
	newPwmData = false;
    } else if(newPidPwm) {
	pwm_data[1].i = leftPidPwm * pidOutK;
	pwm_data[2].i = rightPidPwm * pidOutK;
	res2 = I2Cdev::writeBytes(I2CADDR_STM32, CMD_MCU_SET_PWM, 11, dataptr1);
	newPidPwm = false;
    }

    // update motors on new cmd_vel message
    if(cmdVelComputeEnabled && newCmdVel) {
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
            dataptr = (uint8_t*)tmp_data;
            dataptr++;
            tmp_data[1].i = leftMotorSpeedRequestPulses;
            tmp_data[2].i = rightMotorSpeedRequestPulses;
            I2Cdev::writeBytes(I2CADDR_STM32, CMD_MCU_SET_VEL, 11, dataptr);
	}

	newCmdVel = false;
    }

    // publish odometry on timer
    if(odomPublishEnabled && (ros::Time::now().toSec() > tfPublisherTimer)) {
	odometry_publish(pn, n, odom_pub, odom_broadcaster);
    }

    //
    // SEND MCU COMMANDS
    //
    res3 = 0;
    if(newCmd) {
      switch(cmd) {
        case CMD_DEBUG_ON:
          displayLoopInfo = true;
          break;
        case CMD_DEBUG_OFF:
          displayLoopInfo = false;
          break;
        default:
          // SEND MCU COMMANDS
	  ROS_INFO("send mcu cmd");
          uint8_t* dataptr2 = (uint8_t*)cmd_data;
          dataptr2++;
          res3 = I2Cdev::writeBytes(I2CADDR_STM32, (uint8_t)cmd, 3, dataptr2);
          if(cmd == CMD_MCU_RESET_ENCODER) {
              encoderLeftPulsesLast = 0;
              encoderRightPulsesLast = 0;
	  } else if(cmd == CMD_MCU_LIDAR_MOTOR_ON) {
	      lidarMotorStatus = true;
          } else if(CMD_MCU_LIDAR_MOTOR_OFF) {
              lidarMotorStatus = false;
	  }

          break;
      }
      newCmd = false;
      cmd = 0;
    }

    //
    // PROCESS BUTTONS
    //

    if(!btnStatus && (ros::Time::now().toSec() > btnTimer)) {

        btnState[BTN_BUP] = !bcm2835_gpio_lev(BTN_PIN_BUP);
        btnState[BTN_BDN] = !bcm2835_gpio_lev(BTN_PIN_BDN);
        btnState[BTN_BLF] = !bcm2835_gpio_lev(BTN_PIN_BLF);
        btnState[BTN_BRT] = !bcm2835_gpio_lev(BTN_PIN_BRT);
        btnState[BTN_BNO] = !bcm2835_gpio_lev(BTN_PIN_BNO);
        btnState[BTN_BOK] = !bcm2835_gpio_lev(BTN_PIN_BOK);

        //printf("%d %d %d %d %d %d\n", btnState[BTN_BUP], btnState[BTN_BDN], btnState[BTN_BLF], btnState[BTN_BRT], btnState[BTN_BNO], btnState[BTN_BOK]);

	// buttons imediate actions
        if(menuOn) {
            if(btnState[BTN_BNO]) {
                lcdBackLight(0);
                lcdBacklightStatus = true;
	        lcdBacklightTimer = ros::Time::now().toSec() + LCD_BACKLIGHT_TIMER;
            }
            if(btnState[BTN_BOK]) {
                menuOn = false;
            }
        } else {
            if(btnState[BTN_BNO]) {
                lcdBackLight(0);
                lcdBacklightStatus = true;
                lcdBacklightTimer = ros::Time::now().toSec() + LCD_BACKLIGHT_TIMER;
            }
            if(btnState[BTN_BOK]) {
                menuOn = true;
            }
        }

	// check for any button pressed
        int ttl = 0;
	for(int f = 0; f < BTN_NUM; f++) {
	    ttl += btnState[f];
	}

        if(ttl == 0) {
	    // no button
       	    btnTimer = ros::Time::now().toSec() + BTN_TIMER;
	} else {
	    // button pressed
            //printf("%d %d %d %d %d %d %d %d\n", menuOn, menu, btnState[BTN_BUP], btnState[BTN_BDN], btnState[BTN_BLF], btnState[BTN_BRT], btnState[BTN_BNO], btnState[BTN_BOK]);
            //btnTimer = ros::Time::now().toSec() + BTN_ON_TIMER;
            btnTimer = ros::Time::now().toSec() + BTN_TIMER;
	    btnStatus = true;
	}
    }

    //
    // PROCESS MENU & LCD UPDATE
    //

    // backlight
    if(lcdBacklightStatus && ros::Time::now().toSec() > lcdBacklightTimer) {
	lcdBackLight(1);
	lcdBacklightStatus = false;
    }

    if(ros::Time::now().toSec() > lcdTimer) {
        if(menuOn) {
            switch(menu) {
	        case 0: lcdMenu(); break;
                case 1: lcdMenu1(); break;
                case 2: lcdMenu2(); break;
            }
        } else {
            lcdHome(pn,n);
            //bcm2835_delay(800);
        }

        //reset button status
	btnStatus = false;
        //for(int f = 0; f < BTN_NUM; f++) {
        //    btnState[f] = 0;
        //}

        lcdTimer = ros::Time::now().toSec() + LCD_TIMER;
    }

    // PROCESS IMU
    if(ros::Time::now().toSec() > imuPublisherTimer) {
	mpu6050_process(pn, n);
	imuPublisherTimer = ros::Time::now().toSec() + IMU_PUBLISHER_TIMER;
    }

    if(displayLoopInfo) {
      ROS_INFO("%d %f %f %d %d %d %d %f %f %f %f %d %d %d", cmd, cmd_vel_x, cmd_vel_z, pwm_data[1].i, pwm_data[2].i, enc_data[0].i, enc_data[1].i, leftSpeedPidInput, rightSpeedPidInput, leftSpeedPidSetPoint, rightSpeedPidSetPoint, res1, res2, res3);
    }

}

int main(int argc, char **argv){

    ros::init(argc, argv, "base5");

    // Allows parameters passed in via <param>
    ros::NodeHandle pn("~");

    // Does not allow parameters being passed in.
    ros::NodeHandle n;

    signal(SIGINT, mySigintHandler);

    ROS_INFO("Starting base node...");

    //
    // SETUP BASE PARAMETERS
    //
    pn.param<int>("frequency", sample_rate, DEFAULT_SAMPLE_RATE_HZ);
    std::cout << "Using sample rate: " << sample_rate << std::endl;

    pn.param<bool>("pid_enabled", pidComputeEnabled, 0);
    std::cout << "pid_enabled: " << pidComputeEnabled << std::endl;

    pn.param<bool>("odom_enabled", odomPublishEnabled, 0);
    std::cout << "odom_enabled: " << odomPublishEnabled << std::endl;

    pn.param<bool>("cmdvel_enabled", cmdVelComputeEnabled, 0);
    std::cout << "cmdvel_enabled: " << cmdVelComputeEnabled << std::endl;

    pn.param<double>("cmdvel_k", cmd_vel_k, 1.0);
    std::cout << "cmdvel_k: " << cmd_vel_k << std::endl;

    // units are m, m/s, radian/s
    pn.param<double>("wheel_rad", wheel_rad, WHEEL_RAD);
    std::cout << "wheel_rad: " << wheel_rad << std::endl;

    pn.param<double>("wheel_sep", wheel_sep, WHEEL_SEP);
    std::cout << "wheel_sep: " << wheel_sep << std::endl;

    pn.param<double>("wheel_encoder_pulses", wheel_encoder_pulses, ENCODER_PULSES);
    std::cout << "wheel_encoder_pulses: " << wheel_encoder_pulses << std::endl;

    // stored for fast calculation
    double wheel_per = 2 * PI * wheel_rad;
    double distancePerPulse = wheel_per / wheel_encoder_pulses;
    double pulses_per_m = (1.0 / wheel_per) * wheel_encoder_pulses;

    std::cout << "wheel_per: " << wheel_per << std::endl;
    std::cout << "distancePerPulse: " << distancePerPulse << std::endl;
    std::cout << "pulses_per_m: " << pulses_per_m << std::endl;

    // SETUP PID PARAMETERS
    pid_ros_params_setup(pn);

    // SETUP IMU PARAMETERS
    mpu6050_ros_params_setup(pn);

    //
    // SETUP MOTORS
    //

    // setup ros publishers
    left_encoder_pub = n.advertise<std_msgs::Int64>("wheel_left", 100);
    right_encoder_pub = n.advertise<std_msgs::Int64>("wheel_right", 100);

    // setup ros subscribers
    ros::Subscriber left_pwm_sub = n.subscribe("motor_left", 100, leftPwmCallback);
    ros::Subscriber right_pwm_sub = n.subscribe("motor_right", 100, rightPwmCallback);

    // assure that all pwm and enc data is zero
    for(int i = 0; i < 8; i++) pwm_data[i].i = 0;
    for(int i = 0; i < 8; i++) enc_data[i].i = 0;
    for(int i = 0; i < 8; i++) tmp_data[i].i = 0;

    // pwm velocity
    pwm_data[0].i = 0;
    pwm_data[1].i = 0;

    //
    // OTHER SETUP
    //

    // general commands subscriber
    ros::Subscriber cmd_sub = n.subscribe("cmd", 10, cmdCallback);
    for(int i = 0; i < 8; i++) cmd_data[i].i = 0;

    // command velocity subscriber
//    if(cmdVelComputeEnabled) {
	ROS_INFO("enable cmd_vel subscribe");
        ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 50, twistCallback);
//    }

    // command motion subscriber
    ros::Subscriber cmd_motion_sub = n.subscribe("cmd_motion", 10, cmdMotionCallback);

    //
    // ODOMETRY 
    //
    odometry_setup();

    //if(odomPublishEnabled) {
        ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
        //odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
        tf::TransformBroadcaster odom_broadcaster;
    //}

    // setup other ros variables
    ros::Rate loop_rate(100); //TODO
    encoderPublisherTimer = ros::Time::now().toSec();
    tfPublisherTimer = ros::Time::now().toSec();

    // setup pid
    leftSpeedPidSetPoint = 0;
    rightSpeedPidSetPoint = 0;
    pidTimer = ros::Time::now().toSec();

    //
    // START I2C
    //

    printf("Initializing I2C...\n");
    I2Cdev::initialize();

    //
    // MPU6050 SETUP
    //
    mpu6050_device_setup();
    mpu6050_ros_publishers_setup(n);

    // imu timer
    imuPublisherTimer = ros::Time::now().toSec();

    //
    // SET OTHERS I2C DEVICES
    //

    // reset encoders
    uint8_t* dataptr = (uint8_t*)cmd_data;
    dataptr++;

    I2Cdev::writeBytes(I2CADDR_STM32, CMD_MCU_RESET_ENCODER, 3, dataptr);

    encoderLeftPulsesLast = 0;
    encoderRightPulsesLast = 0;

    // stop lidar motor
    dataptr = (uint8_t*)tmp_data;
    dataptr++;

    I2Cdev::writeBytes(I2CADDR_STM32, CMD_MCU_LIDAR_MOTOR_OFF, 3, dataptr);
    lidarMotorStatus = false;

    //
    // PUSH BUTTONS
    //

    // Set RPI pins to be an input
    bcm2835_gpio_fsel(BTN_PIN_BUP, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(BTN_PIN_BDN, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(BTN_PIN_BLF, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(BTN_PIN_BRT, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(BTN_PIN_BNO, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(BTN_PIN_BOK, BCM2835_GPIO_FSEL_INPT);

    // set pins with a pullup
    bcm2835_gpio_set_pud(BTN_PIN_BUP, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(BTN_PIN_BDN, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(BTN_PIN_BLF, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(BTN_PIN_BRT, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(BTN_PIN_BNO, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(BTN_PIN_BOK, BCM2835_GPIO_PUD_UP);

    btnTimer = ros::Time::now().toSec();

    //
    // LCD
    //

    // set the nokia 5110 lcd pins on bcm2835
    lcdCreate(LCD_PIN_RESET, LCD_PIN_SCE, LCD_PIN_DC, LCD_PIN_SDIN, LCD_PIN_SCLK, LCD_PIN_BL);

    // start the lcd
    lcdInit();

    // disable backlight
    lcdBackLight (1);

    // set the lcd update timer
    lcdTimer = ros::Time::now().toSec();

    ros::Rate r(sample_rate); //TODO
    while(ros::ok()){
        loop(pn, n, odom_pub, odom_broadcaster);
        ros::spinOnce();
        r.sleep(); //TODO
    }

    std::cout << "Shutdown." << std::endl << std::flush;

    return 0;

}
