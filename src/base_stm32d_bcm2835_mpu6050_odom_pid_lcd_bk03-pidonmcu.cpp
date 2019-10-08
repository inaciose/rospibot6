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

// ip address
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>

// uptime
#include <sys/sysinfo.h>

// shutdown and reboot
//#include <sys/reboot.h>

// i2c
#include "I2Cdev.h"

// mpu6050 DMP
#include "MPU6050_6Axis_MotionApps20.h"

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
#define ENCODER_PULSES 1920.0
#define WHEEL_RAD 0.032
#define WHEEL_SEP 0.190

#define PI 3.141592653

#define ENCODER_PUBLISHER_TIMER 0.02
#define TF_PUBLISHER_TIMER 0.02

// units are m, m/s, radian/s
double wheel_encoder_pulses = ENCODER_PULSES;
double wheel_rad = WHEEL_RAD;
double wheel_sep =  WHEEL_SEP;

// stored for fast calculation
double wheel_per = 2 * PI * wheel_rad;
double distancePerPulse = wheel_per / wheel_encoder_pulses;
double pulses_per_m = 1.0 / wheel_per * wheel_encoder_pulses;

// left encoder variables
long encoderLeftPulses;
long encoderRightPulses;

long encoderLeftPulsesTarget = 0;
long encoderRightPulsesTarget = 0;
long encoderLeftPulsesTargetStart = 0;
long encoderRightPulsesTargetStart = 0;

bool encoderLeftPulsesOnTarget = false;
bool encoderRightPulsesOnTarget = false;
bool encoderPulsesTargetEnabled = false;

int encoderLeftTargetDirection = 0;
int encoderRightTargetDirection = 0;

// twist, pwm & pid variables
double cmd_vel_x, cmd_vel_z;
double leftMotorSpeedRequest,rightMotorSpeedRequest;
int leftMotorSpeedRequestPulses, rightMotorSpeedRequestPulses;
int maxSpeedPulses = 45; //TODO
double cmd_vel_k = 1.0;
double odom_pose_covariance, odom_pose_stdev_;
double odom_twist_covariance, odom_twist_stdev_;

long encoderLeftPulsesLast;
long encoderRightPulsesLast;

long encoderLeftSpeedPidPulses;
long encoderRightSpeedPidPulses;

double leftSpeedPidInput, rightSpeedPidInput;
double leftSpeedPidSetPoint, rightSpeedPidSetPoint;

double pidTimer;
bool newPidPwm = false;
double leftPidPwm, rightPidPwm;

//PID constants
#define PID_SETPOINT_MAX 100
#define PID_OUT_TRH 0
#define PID_OUT_MIN 0
#define PID_OUT_MAX 32000
#define PID_OUT_K 1
#define PID_SAMPLE_TIME 0.025

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

// control variables
bool newCmdVel = false;
double encoderPublisherTimer;
double tfPublisherTimer;

// odometry
double bodyX;
double bodyY;
double bodyTheta;

// odometry encoder variables
long encoderLeftPulsesOdomLast;
long encoderRightPulsesOdomLast;

// velocity
double odomXvel = 0;
double odomZvel = 0;

//
// MPU6050
//

#define IMU_PUBLISHER_TIMER 0.05
double imuPublisherTimer;

//Typically, motion processing algorithms should be run at a high rate, often around 200Hz,
//in order to provide accurate results with low latency. This is required even if the application
//updates at a much lower rate; for example, a low power user interface may update as slowly
//as 5Hz, but the motion processing should still run at 200Hz.
//Page 25 of MPU6050 datasheet.
#define DEFAULT_SAMPLE_RATE_HZ	10

#define MPU_FRAMEID "base_imu"

//#include "AccelGyroSensorOffsets.h"

ros::Publisher imu_calib_pub;

ros::ServiceClient * clientptr;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL

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
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int sample_rate;
std::string frame_id;

// mpu offsets
int ax, ay, az, gx, gy, gz;

bool debug = false;

// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
bool ado = false;

double angular_velocity_covariance, pitch_roll_covariance, yaw_covariance, linear_acceleration_covariance;
double linear_acceleration_stdev_, angular_velocity_stdev_, yaw_stdev_, pitch_roll_stdev_;

ros::Publisher imu_pub;
ros::Publisher imu_euler_pub;
ros::Publisher mag_pub;

//
// MOTORS
//
#define MAX_ENCODER_DIFF 5000

#define I2CADDR_STM32 0x0b
#define I2C_MAX_LEN 32
uint8_t i2c_buf[I2C_MAX_LEN];

// data trasfer variables
union u_tag {
    uint8_t b[4];
    int32_t i;
};

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

#define CMD_MCU_RESET_ENCODER 1
#define CMD_MCU_SET_PWM 2
#define CMD_MCU_LIDAR_MOTOR_ON 3
#define CMD_MCU_LIDAR_MOTOR_OFF 4
#define CMD_MCU_SET_MOTION 5
#define CMD_MCU_SET_VEL 6
#define CMD_DEBUG_ON 100
#define CMD_DEBUG_OFF 99

union u_tag cmd_data[8];
uint16_t cmd = 0;
bool newCmd = false;

bool lidarMotorStatus = false;

bool newCmdMotion = false;
int cmdMotion[64];

// loop info for debug
bool displayLoopInfo = false;

void getIfAddress(char* ifname, char* result) {
        int fd;
        struct ifreq ifr;

        // open soket
        fd = socket(AF_INET, SOCK_DGRAM, 0);
        // get an IPv4 IP address
        ifr.ifr_addr.sa_family = AF_INET;
        // get IP address attached to ifname: ex. "eth0" or "wlx0013efcb0cbc"
        strncpy(ifr.ifr_name, ifname, IFNAMSIZ-1);
        ioctl(fd, SIOCGIFADDR, &ifr);
        close(fd);
        // convert to human readable
        sprintf(result, "%s", inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
        // debug
        //printf("%s %s\n", ifname, result);
}

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
    mpu.reset();

    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

double computeLeftPID(double inp) {
    ros::Time previousTime = ros::Time::now();
    double elapsedTime;
    double error;
    double rateError;
    static double lastError = 0;
    static double cumError = 0;

    //currentTime = millis();
    ros::Time currentTime = ros::Time::now();
    elapsedTime = (double)(currentTime.toSec() - previousTime.toSec());

    error = leftSpeedPidSetPoint - inp; // compute proporcional
    cumError += error * elapsedTime; // compute integral
    rateError = (error - lastError)/elapsedTime; // compute derivative

    double out = kpLeft*error + kiLeft*cumError + kdLeft*rateError;

    //ROS_INFO("PID %f %f %f %f %f %f", error, cumError, rateError, elapsedTime, inp, out);

    if(out > pidOutTrh) out += pidOutMin;
    if(out > pidOutMax) out = pidOutMax;
    if(out < 0) out = 0;

    lastError = error;
    previousTime = currentTime;

    return out;
}

double computeRightPID(double inp) {
    ros::Time previousTime = ros::Time::now();
    double elapsedTime;
    double error;
    double rateError;
    static double lastError = 0;
    static double cumError = 0;

    ros::Time currentTime = ros::Time::now();
    elapsedTime = (double)(currentTime.toSec() - previousTime.toSec());

    error = rightSpeedPidSetPoint - inp; // compute proporcional
    cumError += error * elapsedTime; // compute integral
    rateError = (error - lastError)/elapsedTime; // compute derivative

    double out = kpRight*error + kiRight*cumError + kdRight*rateError;

    if(out > pidOutTrh) out += pidOutMin;
    if(out > pidOutMax) out = pidOutMax;
    if(out < 0) out = 0;
    lastError = error;
    previousTime = currentTime;

    return out;
}

void lcdHome(ros::NodeHandle pn, ros::NodeHandle n) {
    // lcd output vars
    char lcdOutString1[24];
    char lcdOutString2[24];
    char lcdOutString3[24];

    // lcd uptime
    struct sysinfo info;
    int uptimeDays = 0;
    int uptimeHours = 0;
    int uptimeMins = 0;
    int uptimeSecs = 0;


        // get the ip addresses
        getIfAddress("eth0", lcdOutString2);
        getIfAddress("wlx0013efcb0cbc", lcdOutString3);

        // get uptime (from system info)
        // we can also get the load
        sysinfo(&info);

        // convert to human standart
        uptimeMins = info.uptime / 60;
        uptimeSecs = info.uptime - uptimeMins * 60;

        uptimeHours = uptimeMins / 60;
        uptimeMins = uptimeMins - uptimeHours * 60;

        uptimeDays = uptimeHours / 24;
        uptimeHours = uptimeHours - uptimeDays * 24;

        // debug
        //printf("Uptime = %ld %d %02d:%02d:%02d\n", info.uptime, uptimeDays, uptimeHours, uptimeMins, upti$

        // display on lcd
        lcdClear();
        sprintf(lcdOutString1, "IP & uptime ");
        lcdGotoXY(0,0);
        lcdString(lcdOutString1);

        lcdGotoXY(0,1);
        lcdString(lcdOutString2);

        lcdGotoXY(0,3);
        lcdString(lcdOutString3);

        lcdGotoXY(0,5);
        sprintf(lcdOutString1,"%d %02d:%02d:%02d", uptimeDays, uptimeHours, uptimeMins, uptimeSecs);
        lcdString(lcdOutString1);
}

void lcdMenuShowItem(uint8_t x, uint8_t y, const char text[20], bool sel) {
        char outString[24];
        if(sel) {
        	sprintf(outString, "> %s", text);
	} else {
                sprintf(outString, "  %s", text);
        }

        lcdGotoXY(x,y);
        lcdString(outString);
}

void lcdMenu1() {

	// menu config
        uint8_t menuid = 1;
        int selected = menuselect[menuid][0];

        // lcd output vars
        char outString[20];

        // mcu command
        uint8_t res;
        uint8_t* dataptr = (uint8_t*)tmp_data;
        dataptr++;

//        printf("+%d %d %d %d %d %d %d %d %d\n", menuOn, menu, selected, btnState[BTN_BUP], btnState[BTN_BDN], btnState[BTN_BLF], btnState[BTN_BRT], btnState[BTN_BNO], $

        // up/down button pressed
        if(btnState[BTN_BUP] && (selected > 0)) selected--;
        if(btnState[BTN_BDN] && (selected < menuselect[menuid][1]-1)) selected++;

        if(btnState[BTN_BUP] || btnState[BTN_BDN]) {
            menuselect[menuid][0] = selected;
            lcdTimer = ros::Time::now().toSec(); // set timer for now
            printf("-%d %d %d\n", menuOn, menu, selected);
        }

        // back button pressed
        if(btnState[BTN_BLF]) {
            menu = 0;
            menuselect[menuid][0] = 0;
            //menuOn = false;
            lcdTimer = ros::Time::now().toSec(); // set timer for now
            return;
        }

        // select buttom pressed
        if(btnState[BTN_BRT]) {
            switch(selected) {
                case 0:
                    menu = 2;
                    return;
                    break;
                case 1:
                    //  send lidar command
                    if(lidarMotorStatus) {
                        ROS_INFO("send lidar off mcu cmd");
                        res = I2Cdev::writeBytes(I2CADDR_STM32, CMD_MCU_LIDAR_MOTOR_OFF, 3, dataptr);
                        lidarMotorStatus = false;
                    } else {
                        ROS_INFO("send lidar on mcu cmd");
                        res = I2Cdev::writeBytes(I2CADDR_STM32, CMD_MCU_LIDAR_MOTOR_ON, 3, dataptr);
                        lidarMotorStatus = true;
                    }
                    break;
                case 2:
                    ROS_INFO("shutdown");
                    break;
            }

            lcdTimer = ros::Time::now().toSec(); // set timer for now
        }


        // display on lcd
        lcdClear();
        sprintf(outString, "--- EXTRA ---");
        lcdGotoXY(0,0);
        lcdString(outString);

        lcdMenuShowItem(0,1, "Shutdown", (selected == 0) ? 1 : 0);
        lcdMenuShowItem(0,2, "Lidar", (selected == 1) ? 1 : 0);
        lcdMenuShowItem(0,3, "Outro", (selected == 2) ? 1 : 0);
}


void lcdMenu2() {
        // menu config
        uint8_t menuid = 2;
        int selected = menuselect[menuid][0];

        // lcd output vars
        char outString[20];

        // mcu command
        uint8_t res;
        uint8_t* dataptr = (uint8_t*)tmp_data;
        dataptr++;

        // up/down button pressed
        if(btnState[BTN_BUP] && (selected > 0)) selected--;
        if(btnState[BTN_BDN] && (selected < menuselect[menuid][1]-1)) selected++;

        if(btnState[BTN_BUP] || btnState[BTN_BDN]) {
            menuselect[menuid][0] = selected;
            lcdTimer = ros::Time::now().toSec(); // set timer for now
        }

        // back button pressed
        if(btnState[BTN_BLF]) {
	    menu = 0;
            menuselect[menuid][0] = 0;
            //menuOn = false;
            lcdTimer = ros::Time::now().toSec(); // set timer for now
            return;
        }

        // select buttom pressed
        if(btnState[BTN_BRT]) {
            switch(selected) {
                case 0:
                    ROS_INFO("shutdown");
		    //sync();
		    //reboot(RB_POWER_OFF);
                    system("shutdown -P now");
                    break;
                case 1:
                    ROS_INFO("reboot");
		    //sync();
		    //reboot(RB_AUTOBOOT);
                    system("reboot");
                    break;
            }
            lcdTimer = ros::Time::now().toSec(); // set timer for now
        }

        // display on lcd
        lcdClear();
        sprintf(outString, "- ON/OFF --");
        lcdGotoXY(0,0);
        lcdString(outString);

        lcdMenuShowItem(0,1, "Shutdown", (selected == 0) ? 1 : 0);
        lcdMenuShowItem(0,2, "Reboot", (selected == 1) ? 1 : 0);

}

void lcdMenu() {

        // menu config
        uint8_t menuid = 0;
        int selected = menuselect[menuid][0];

        // lcd output vars
        char outString[20];

        // mcu command
        uint8_t res;
        uint8_t* dataptr = (uint8_t*)tmp_data;
        dataptr++;

	printf("+%d %d %d %d %d %d %d %d %d\n", menuOn, menu, selected, btnState[BTN_BUP], btnState[BTN_BDN], btnState[BTN_BLF], btnState[BTN_BRT], btnState[BTN_BNO], btnState[BTN_BOK]);

	// up/down button pressed
        if(btnState[BTN_BUP] && (selected > 0)) selected--;
        if(btnState[BTN_BDN] && (selected < menuselect[menuid][1]-1)) selected++;

        if(btnState[BTN_BUP] || btnState[BTN_BDN]) {
            menuselect[menuid][0] = selected;
            lcdTimer = ros::Time::now().toSec(); // set timer for now
	    printf("-%d %d %d\n", menuOn, menu, selected);
        }

	// back button pressed
        if(btnState[BTN_BLF]) {
            menuselect[menuid][0] = 0;
	    menuOn = false;
	    lcdTimer = ros::Time::now().toSec(); // set timer for now
	    return;
        }

	// select buttom pressed
        if(btnState[BTN_BRT]) {
	    switch(selected) {
		case 0:
                    menu = 1;
		    return;
		    break;
		case 1:
		    //  send lidar command
		    if(lidarMotorStatus) {
			ROS_INFO("send lidar off mcu cmd");
          	        res = I2Cdev::writeBytes(I2CADDR_STM32, CMD_MCU_LIDAR_MOTOR_OFF, 3, dataptr);
			lidarMotorStatus = false;
		    } else {
                        ROS_INFO("send lidar on mcu cmd");
                        res = I2Cdev::writeBytes(I2CADDR_STM32, CMD_MCU_LIDAR_MOTOR_ON, 3, dataptr);
                        lidarMotorStatus = true;
		    }
                    break;
                case 2:
		    ROS_INFO("shutdown");
                    menu = 2;
                    return;
                    break;
	    }
	    lcdTimer = ros::Time::now().toSec(); // set timer for now
        }

        // display on lcd
        lcdClear();
        sprintf(outString, "--- Home ---");
        lcdGotoXY(0,0);
        lcdString(outString);

        lcdMenuShowItem(0,1, "Menu 1", (selected == 0) ? 1 : 0);
        lcdMenuShowItem(0,2, "Lidar", (selected == 1) ? 1 : 0);
	lcdMenuShowItem(0,3, "Shutdown", (selected == 2) ? 1 : 0);
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

	// read encoders from slave
	for (i = 0; i < I2C_MAX_LEN; i++) i2c_buf[i] = 0;
	res1 = I2Cdev::readBytes(I2CADDR_STM32, 0, 12, i2c_buf);

	// load values to int's
	tmp_data[0].b[0] = i2c_buf[0+4];
	tmp_data[0].b[1] = i2c_buf[1+4];
	tmp_data[0].b[2] = i2c_buf[2+4];
	tmp_data[0].b[3] = i2c_buf[3+4];
	tmp_data[1].b[0] = i2c_buf[4+4];
	tmp_data[1].b[1] = i2c_buf[5+4];
	tmp_data[1].b[2] = i2c_buf[6+4];
	tmp_data[1].b[3] = i2c_buf[7+4];

	// try to filter some strange noise (values) from encoder readings
	if(abs(tmp_data[0].i) - abs(enc_data[0].i) < MAX_ENCODER_DIFF ) {
	    enc_data[0].i = tmp_data[0].i;
	}

	if(abs(tmp_data[1].i) - abs(enc_data[1].i) < MAX_ENCODER_DIFF ) {
	    enc_data[1].i = tmp_data[1].i;
	}

        // set some vars for debug
        if(encoderPulsesTargetEnabled) {
            // check left encoder target
            if(!encoderLeftPulsesOnTarget) {
                if(encoderLeftTargetDirection >= 0) {
                    if(encoderLeftPulses >= encoderLeftPulsesTarget) {
                        encoderLeftPulsesOnTarget = true;
                    }
                } else {
                    if(encoderLeftPulses < encoderLeftPulsesTarget) {
                        encoderLeftPulsesOnTarget = true;
                    }
                }
                // left stop on encoder target
                if(encoderLeftPulsesOnTarget) {
                    ROS_INFO("L ON TARGET");
                    //leftMotorPwmOut = 0;
                    //leftSpeedPidSetPoint = 0;
                    //leftSpeedPidSetPointDirection = 0;
                }
            }

            // check right encoder target
            if(!encoderRightPulsesOnTarget) {
                if(encoderRightTargetDirection >= 0) {
                    if(encoderRightPulses >= encoderRightPulsesTarget) {
                        encoderRightPulsesOnTarget = true;
                    }
                } else {
                    if(encoderRightPulses < encoderRightPulsesTarget) {
                        encoderRightPulsesOnTarget = true;
                    }
                }
                // right stop on encoder target
                if(encoderRightPulsesOnTarget) {
                    ROS_INFO("R ON TARGET");
                    //rightMotorPwmOut = 0;
                    //rightSpeedPidSetPoint = 0;
                    //rightSpeedPidSetPointDirection = 0;
                }
            }

            // encoders on target
            if(encoderLeftPulsesOnTarget && encoderRightPulsesOnTarget) {
                ROS_INFO("BOTH ON TARGET1 %d %d %d %d %d", 
                    (encoderLeftPulses - encoderLeftPulsesTargetStart) - (encoderRightPulses - encoderRightPulsesTargetStart),
                    (encoderLeftPulses - encoderLeftPulsesTargetStart),
                    (encoderRightPulses - encoderRightPulsesTargetStart),
                    encoderLeftPulses, encoderRightPulses);

                delay(125);

                ROS_INFO("BOTH ON TARGET2 %d %d %d %d %d", 
                    (encoderLeftPulses - encoderLeftPulsesTargetStart) - (encoderRightPulses - encoderRightPulsesTargetStart),
                    (encoderLeftPulses - encoderLeftPulsesTargetStart),
                    (encoderRightPulses - encoderRightPulsesTargetStart),
                    encoderLeftPulses, encoderRightPulses);


                encoderLeftPulsesTarget = 0;
                encoderLeftPulsesOnTarget = false;
                encoderRightPulsesTarget = 0;
                encoderRightPulsesOnTarget = false;
                encoderPulsesTargetEnabled = false;
                encoderLeftPulsesTargetStart = 0;
                encoderRightPulsesTargetStart = 0;
                encoderLeftTargetDirection = 0;
                encoderRightTargetDirection = 0;

                }
        }

	// set some vars for pid
	int encoderLeftUpdate = enc_data[0].i - encoderLeftPulsesLast;
	int encoderRightUpdate = enc_data[1].i - encoderRightPulsesLast;
	encoderLeftPulsesLast = enc_data[0].i;
	encoderRightPulsesLast = enc_data[1].i;
	encoderLeftSpeedPidPulses += encoderLeftUpdate;
	encoderRightSpeedPidPulses += encoderRightUpdate;

	// set some vars for odom
	encoderLeftPulses = enc_data[0].i;
	encoderRightPulses = enc_data[1].i;

	//printf("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n", res1, i2c_buf[0], i2c_buf[1], i2c_buf[2], i2c_buf[3], i2c_buf[4], i2c_buf[5], i2c_buf[6], i2c_buf[7], i2c_buf[8], i2c_buf[9], i2c_buf[10], i2c_buf[11], enc_data[0].i, enc_data[1].i);

	// publish left encoder ros topic
	msgEnc.data = enc_data[0].i;
	left_encoder_pub.publish(msgEnc);

	// publish right encoder  ros topic
	msgEnc.data = enc_data[1].i;
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

	current_time = ros::Time::now();
	double elapsed = (last_time - current_time).toSec();
	last_time  = current_time;

	//float d_left = (encoderLeftPulses - encoderLeftPulsesOdomLast) * distancePerPulse;
	//float d_right = (encoderRightPulses - encoderRightPulsesOdomLast) * distancePerPulse;

        float d_left = (encoderLeftPulses - encoderLeftPulsesOdomLast) / pulses_per_m;
        float d_right = (encoderRightPulses - encoderRightPulsesOdomLast) / pulses_per_m;

	encoderLeftPulsesOdomLast = encoderLeftPulses;
	encoderRightPulsesOdomLast = encoderRightPulses;

	// distance traveled is the average of the two wheels
	float d = ( d_left + d_right ) / 2;

	// this approximation works (in radians) for small angles
	float th = ( d_right - d_left ) / wheel_sep;

	// calculate velocities
	odomXvel = d / elapsed;
	odomZvel = th / elapsed;

	if(d != 0) {
	    // calculate distance traveled in x and y
	    double x = cos(th) * d;
	    double y = -sin(th) * d;
	    //calculate the final position of the robot
	    bodyX = bodyX + (cos(bodyTheta) * x - sin(bodyTheta) * y);
	    bodyY = bodyY + (sin(bodyTheta) * x + cos(bodyTheta) * y);
	}

	if(th != 0) {
	    bodyTheta = bodyTheta + th;
	}

	//ROS_INFO("odom: %f %f %f %f %f %f %f", bodyX, bodyY, bodyTheta, d_left, d_right, d, th);

	//since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(bodyTheta);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = bodyX;
	odom_trans.transform.translation.y = bodyY;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//send the transform
	odom_broadcaster.sendTransform(odom_trans);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";

	//set the position
	odom.pose.pose.position.x = bodyX;
	odom.pose.pose.position.y = bodyY;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	// non zero covariance diagonal for efk
        odom.pose.covariance[0] = odom_pose_covariance;
        odom.pose.covariance[7] = odom_pose_covariance;
        odom.pose.covariance[14] = odom_pose_covariance;
        odom.pose.covariance[21] = odom_pose_covariance;
        odom.pose.covariance[28] = odom_pose_covariance;
        odom.pose.covariance[35] = odom_pose_covariance;

	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = odomXvel;
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.angular.z = odomZvel;

        // non zero covariance diagonal for efk
        odom.twist.covariance[0] = odom_twist_covariance;
        odom.twist.covariance[7] = odom_twist_covariance;
        odom.twist.covariance[14] = odom_twist_covariance;
        odom.twist.covariance[21] = odom_twist_covariance;
        odom.twist.covariance[28] = odom_twist_covariance;
        odom.twist.covariance[35] = odom_twist_covariance;

	//publish the message
	odom_pub.publish(odom);
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

    //
    // PROCESS IMU
    //

    if(ros::Time::now().toSec() > imuPublisherTimer) {

	// if programming failed, don't try to do anything
	if (!dmpReady) return;

	ros::Time now = ros::Time::now();

	//http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Imu.html
	sensor_msgs::Imu imu_msg;
	imu_msg.header.stamp = now;
	imu_msg.header.frame_id = frame_id;

	geometry_msgs::Vector3Stamped imu_euler_msg;
	imu_euler_msg.header.stamp = now;
	imu_euler_msg.header.frame_id = frame_id;

	geometry_msgs::Vector3Stamped mag_msg;
	mag_msg.header.stamp = now;
	mag_msg.header.frame_id = frame_id;

	// http://www.i2cdevlib.com/forums/topic/4-understanding-raw-values-of-accelerometer-and-gyrometer/
	    //The output scale for any setting is [-32768, +32767] for each of the six axes.
	//The default setting in the I2Cdevlib class is +/- 2g for the accel and +/- 250 deg/sec
	//for the gyro. If the device is perfectly level and not moving, then:
	    //
	    //    X/Y accel axes should read 0
	    //    Z accel axis should read 1g, which is +16384 at a sensitivity of 2g
	    //    X/Y/Z gyro axes should read 0
	    //
	    //In reality, the accel axes won't read exactly 0 since it is difficult to be perfectly level
	//and there is some noise/error, and the gyros will also not read exactly 0 for the same
	//reason (noise/error).

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	if (fifoCount == 1024) {

	    // reset so we can continue cleanly
	    mpu.resetFIFO();
	    if(debug) printf("FIFO overflow!\n");

	// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} else if (fifoCount >= 42) {

	    // read a packet from FIFO
	    mpu.getFIFOBytes(fifoBuffer, packetSize);

	    // display quaternion values in easy matrix form: w x y z
	    mpu.dmpGetQuaternion(&q, fifoBuffer);
	    if(debug) printf("quat %7.2f %7.2f %7.2f %7.2f    ", q.w,q.x,q.y,q.z);

	    imu_msg.orientation.x = q.x;
	    imu_msg.orientation.y = q.y;
	    imu_msg.orientation.z = q.z;
	    imu_msg.orientation.w = q.w;

	    imu_msg.linear_acceleration_covariance[0] = linear_acceleration_covariance;
	    imu_msg.linear_acceleration_covariance[4] = linear_acceleration_covariance;
	    imu_msg.linear_acceleration_covariance[8] = linear_acceleration_covariance;

	    imu_msg.angular_velocity_covariance[0] = angular_velocity_covariance;
	    imu_msg.angular_velocity_covariance[4] = angular_velocity_covariance;
	    imu_msg.angular_velocity_covariance[8] = angular_velocity_covariance;

	    imu_msg.orientation_covariance[0] = pitch_roll_covariance;
	    imu_msg.orientation_covariance[4] = pitch_roll_covariance;
	    imu_msg.orientation_covariance[8] = yaw_covariance;

	    #ifdef OUTPUT_READABLE_YAWPITCHROLL
		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

		// Should be in rad/sec.
		imu_msg.angular_velocity.x = ypr[2];
		imu_msg.angular_velocity.y = ypr[1];
		imu_msg.angular_velocity.z = ypr[0];

		if(debug) printf("ypr (degrees)  %7.2f %7.2f %7.2f    ", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
	    #endif

	    #ifdef OUTPUT_READABLE_REALACCEL
		// display real acceleration, adjusted to remove gravity
		// https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050_6Axis_MotionApps20.h
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

		// By default, accel is in arbitrary units with a scale of 16384/1g.
		// Per http://www.ros.org/reps/rep-0103.html
		// and http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
		// should be in m/s^2.
		// 1g = 9.80665 m/s^2, so we go arbitrary -> g -> m/s^s
		imu_msg.linear_acceleration.x = aaReal.x * 1/16384. * 9.80665;
		imu_msg.linear_acceleration.y = aaReal.y * 1/16384. * 9.80665;
		imu_msg.linear_acceleration.z = aaReal.z * 1/16384. * 9.80665;

		if(debug) printf("areal (raw) %6d %6d %6d    ", aaReal.x, aaReal.y, aaReal.z);
		if(debug) printf("areal (m/s^2) %6d %6d %6d    ", imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
	    #endif

	    imu_pub.publish(imu_msg);

	    if(debug) printf("\n");
	}

	imuPublisherTimer = ros::Time::now().toSec() + IMU_PUBLISHER_TIMER;
    }

    if(displayLoopInfo) {
      ROS_INFO("%d %f %f %d %d %d %d %f %f %f %f %d %d %d", cmd, cmd_vel_x, cmd_vel_z, pwm_data[1].i, pwm_data[2].i, enc_data[0].i, enc_data[1].i, leftSpeedPidInput, rightSpeedPidInput, leftSpeedPidSetPoint, rightSpeedPidSetPoint, res1, res2, res3);
    }

}

int main(int argc, char **argv){

    ros::init(argc, argv, "base4");

    // Allows parameters passed in via <param>
    ros::NodeHandle pn("~");

    // Does not allow parameters being passed in.
    ros::NodeHandle n;

    signal(SIGINT, mySigintHandler);

    ROS_INFO("Starting base node...");

    //
    // SETUP BASE PARAMETERS
    //

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

    //
    // SETUP PID PARAMETERS
    //

    pn.param<double>("pid_out_trh", pidOutTrh, PID_OUT_TRH);
    std::cout << "pid_out_trh: " << pidOutTrh << std::endl;

    pn.param<double>("pid_out_min", pidOutMin, PID_OUT_MIN);
    std::cout << "pid_out_min: " << pidOutMin << std::endl;

    pn.param<double>("pid_out_max", pidOutMax, PID_OUT_MAX );
    std::cout << "pid_out_max: " << pidOutMax << std::endl;

    pn.param<double>("pid_out_k", pidOutK, PID_OUT_K);
    std::cout << "pid_out_k: " << pidOutK << std::endl;

    pn.param<double>("pid_sample_time", pidSampleTime, PID_SAMPLE_TIME);
    std::cout << "pid_sample_time: " << pidSampleTime << std::endl;

    pn.param<double>("pid_setpoint_max", pidSetPointMax, PID_SETPOINT_MAX);
    std::cout << "pid_setpoint_max: " << pidSetPointMax << std::endl;

    pn.param<double>("pid_left_kp", kpLeft, 100);
    std::cout << "pid_left_kp: " << kpLeft << std::endl;

    pn.param<double>("pid_left_ki", kiLeft, 0.1);
    std::cout << "pid_left_ki: " << kiLeft << std::endl;

    pn.param<double>("pid_left_kd", kdLeft, 0);
    std::cout << "pid_left_kd: " << kdLeft << std::endl;

    pn.param<double>("pid_right_kp", kpRight, 100);
    std::cout << "pid_right_kp: " << kpRight << std::endl;

    pn.param<double>("pid_right_ki", kiRight, 0.1);
    std::cout << "pid_right_ki: " << kiRight << std::endl;

    pn.param<double>("pid_right_kd", kdRight, 0.0);
    std::cout << "pid_right_kd: " << kdRight << std::endl;

    //
    // SETUP IMU PARAMETERS
    //

    //TODO
    pn.param<int>("frequency", sample_rate, DEFAULT_SAMPLE_RATE_HZ);
    std::cout << "Using sample rate: " << sample_rate << std::endl;

    pn.param<std::string>("frame_id", frame_id, MPU_FRAMEID);
    std::cout << "Using frame_id: " << frame_id << std::endl;

    pn.param<int>("ax", ax, 0);
    pn.param<int>("ay", ay, 0);
    pn.param<int>("az", az, 0);
    pn.param<int>("gx", gx, 0);
    pn.param<int>("gy", gy, 0);
    pn.param<int>("gz", gz, 0);

    pn.param<bool>("ado", ado, false);
    std::cout << "ADO: " << ado << std::endl << std::flush;

    pn.param<bool>("debug", debug, false);
    std::cout << "Debug: " << debug << std::endl << std::flush;

    // NOISE PERFORMANCE: Power Spectral Density @10Hz, AFS_SEL=0 & ODR=1kHz 400 ug/√Hz (probably wrong)
    pn.param("linear_acceleration_stdev", linear_acceleration_stdev_, (400 / 1000000.0) * 9.807 );

    // Total RMS Noise: DLPFCFG=2 (100Hz) 0.05 º/s-rms (probably lower (?) @ 42Hz)
    pn.param("angular_velocity_stdev", angular_velocity_stdev_, 0.05 * (M_PI / 180.0));

    // 1 degree for pitch and roll
    pn.param("pitch_roll_stdev", pitch_roll_stdev_, 1.0 * (M_PI / 180.0));

    // 5 degrees for yaw
    pn.param("yaw_stdev", yaw_stdev_, 5.0 * (M_PI / 180.0));

    angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_;
    linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_;
    pitch_roll_covariance = pitch_roll_stdev_ * pitch_roll_stdev_;
    yaw_covariance = yaw_stdev_ * yaw_stdev_;

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
    odom_pose_stdev_ = 0.2;
    odom_twist_stdev_ = 0.2;

    odom_pose_covariance = odom_pose_stdev_ * odom_pose_stdev_;
    odom_twist_covariance = odom_twist_stdev_ * odom_twist_stdev_;

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

    // verify connection
    printf("Testing device connections...\n");
    mpu = MPU6050(ado ? 0x69 : 0x68);
    if(mpu.testConnection()){
        std::cout << "MPU6050 connection successful" << std::endl << std::flush;
    }else{
        std::cout << "MPU6050 connection failed" << std::endl << std::flush;
        return 1;
    }

    // initialize device
    printf("Initializing I2C devices...\n");
    mpu.initialize();

    // load and configure the DMP
    printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();

    // Set accel offsets.
    std::cout << "Setting X accel offset: " << ax << std::endl;
    mpu.setXAccelOffset(ax);
    std::cout << "Setting Y accel offset: " << ay << std::endl;
    mpu.setYAccelOffset(ay);
    std::cout << "Setting Z accel offset: " << az << std::endl;
    mpu.setZAccelOffset(az);

    // Set gyro offsets.
    std::cout << "Setting X gyro offset: " << gx << std::endl;
    mpu.setXGyroOffset(gx);
    std::cout << "Setting Y gyro offset: " << gy << std::endl;
    mpu.setYGyroOffset(gy);
    std::cout << "Setting Z gyro offset: " << gz << std::endl;
    mpu.setZGyroOffset(gz);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
    }

    usleep(100000);

    imu_pub = n.advertise<sensor_msgs::Imu>("imu/data", 10);
    imu_euler_pub = n.advertise<geometry_msgs::Vector3Stamped>("imu/euler", 10);
    mag_pub = n.advertise<geometry_msgs::Vector3Stamped>("imu/mag", 10);

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
