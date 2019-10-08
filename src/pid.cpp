#include <ros/ros.h>

#include "pid_def.h"

// globals defined localy
long encoderLeftSpeedPidPulses;
long encoderRightSpeedPidPulses;

double leftSpeedPidInput;
double rightSpeedPidInput;
double leftSpeedPidSetPoint;
double rightSpeedPidSetPoint;
double pidOutK = PID_OUT_K;

// locals
double pidOutTrh = PID_OUT_TRH;
double pidOutMin = PID_OUT_MIN;
double pidOutMax = PID_OUT_MAX;
double pidSampleTime = PID_SAMPLE_TIME;
double pidSetPointMax =  PID_SETPOINT_MAX;

double kpLeft = 1;
double kiLeft = 0.0005;
double kdLeft = 0;

double kpRight = 1;
double kiRight = 0.0005;
double kdRight = 0;

//
// SETUP PID PARAMETERS FROM ROS LAUNCH FILE VALUES
//

void pid_ros_params_setup(ros::NodeHandle pn) {
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

}

//
// COMPUTE PID
//


double pid_compute_left(double inp) {
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

double pid_compute_right(double inp) {
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
