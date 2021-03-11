#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// i2c
#include "I2Cdev.h"

#include "mcu.h"
#include "odometry_def.h"

#include "globals.h"
#include "odometry_globals.h"
#include "pid_globals.h"

double wheel_encoder_pulses = ENCODER_PULSES;
double wheel_rad = WHEEL_RAD;
double wheel_sep =  WHEEL_SEP;

// stored for fast calculation
double wheel_per = 2 * PI * wheel_rad;
double distancePerPulse = wheel_per / wheel_encoder_pulses;
double pulses_per_m = 1.0 / wheel_per * wheel_encoder_pulses;

// global encoder variables
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

long encoderLeftPulsesLast;
long encoderRightPulsesLast;

// local odometry variables
double odom_pose_covariance, odom_pose_stdev_;
double odom_twist_covariance, odom_twist_stdev_;

// local odometry encoder variables
long encoderLeftPulsesOdomLast;
long encoderRightPulsesOdomLast;

// global odometry variables
double bodyX;
double bodyY;
double bodyTheta;

// global odometry velocity
double odomXvel = 0;
double odomZvel = 0;

// debug
int mcuSteeringPidError;
int mcuLeftPidError;
int mcuRightPidError;

void read_i2c_encoders() {
	int i;
    	uint8_t res1;
	uint8_t i2c_buf[I2C_MAX_LEN];
        // read encoders from slave
        for (i = 0; i < I2C_MAX_LEN; i++) i2c_buf[i] = 0;
        //res1 = I2Cdev::readBytes(I2CADDR_STM32, 0, 12, i2c_buf);
        res1 = I2Cdev::readBytes(I2CADDR_STM32, 0, 24, i2c_buf);

        // load values to int's
        tmp_data[0].b[0] = i2c_buf[0+4];
        tmp_data[0].b[1] = i2c_buf[1+4];
        tmp_data[0].b[2] = i2c_buf[2+4];
        tmp_data[0].b[3] = i2c_buf[3+4];
        tmp_data[1].b[0] = i2c_buf[4+4];
        tmp_data[1].b[1] = i2c_buf[5+4];
        tmp_data[1].b[2] = i2c_buf[6+4];
        tmp_data[1].b[3] = i2c_buf[7+4];

        tmp_data[2].b[0] = i2c_buf[8+4];
        tmp_data[2].b[1] = i2c_buf[9+4];
        tmp_data[2].b[2] = i2c_buf[10+4];
        tmp_data[2].b[3] = i2c_buf[11+4];
        tmp_data[3].b[0] = i2c_buf[12+4];
        tmp_data[3].b[1] = i2c_buf[13+4];
        tmp_data[3].b[2] = i2c_buf[14+4];
        tmp_data[3].b[3] = i2c_buf[15+4];
        tmp_data[4].b[0] = i2c_buf[16+4];
        tmp_data[4].b[1] = i2c_buf[17+4];
        tmp_data[4].b[2] = i2c_buf[18+4];
        tmp_data[4].b[3] = i2c_buf[19+4];

        // try to filter some strange noise (values) from encoder readings
        if(abs(tmp_data[0].i) - abs(enc_data[0].i) < MAX_ENCODER_DIFF ) {
            enc_data[0].i = tmp_data[0].i;
        }

        if(abs(tmp_data[1].i) - abs(enc_data[1].i) < MAX_ENCODER_DIFF ) {
            enc_data[1].i = tmp_data[1].i;
        }

        // set some vars for odom
        encoderLeftPulses = enc_data[0].i;
        encoderRightPulses = enc_data[1].i;

	// set some var por mcu pid debug
	mcuLeftPidError = enc_data[2].i;
        mcuRightPidError = enc_data[3].i;
        mcuSteeringPidError = enc_data[4].i;

        // set some vars for pid
        int encoderLeftUpdate = enc_data[0].i - encoderLeftPulsesLast;
        int encoderRightUpdate = enc_data[1].i - encoderRightPulsesLast;
        encoderLeftPulsesLast = enc_data[0].i;
        encoderRightPulsesLast = enc_data[1].i;
        encoderLeftSpeedPidPulses += encoderLeftUpdate;
        encoderRightSpeedPidPulses += encoderRightUpdate;

	// debug
        int leftTmpPulses = (encoderLeftPulses - encoderLeftPulsesTargetStart);
	int rightTmpPulses = (encoderRightPulses - encoderRightPulsesTargetStart);

	ROS_INFO("BOTH ON TARGET1 %d %d %d %d %d %d %d", mcuLeftPidError, mcuRightPidError, mcuSteeringPidError, encoderLeftPulses, encoderRightPulses, leftTmpPulses, rightTmpPulses);

	//return res1;

/*
        // publish left encoder ros topic
        msgEnc.data = enc_data[0].i;
        left_encoder_pub.publish(msgEnc);

        // publish right encoder  ros topic
        msgEnc.data = enc_data[1].i;
        right_encoder_pub.publish(msgEnc);
*/

}

void check_encoders_position() {

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


}

void odometry_setup() {
    odom_pose_stdev_ = 0.2;
    odom_twist_stdev_ = 0.2;

    odom_pose_covariance = odom_pose_stdev_ * odom_pose_stdev_;
    odom_twist_covariance = odom_twist_stdev_ * odom_twist_stdev_;
}

/*
void odometry_ros_setup(ros::NodeHandle n, tf::TransformBroadcaster odom_broadcaster) {
        ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
        tf::TransformBroadcaster odom_broadcaster;

}
*/

void odometry_publish(ros::NodeHandle pn, ros::NodeHandle n, ros::Publisher odom_pub, tf::TransformBroadcaster odom_broadcaster) {
        ros::Time current_time;
        static ros::Time last_time;

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
