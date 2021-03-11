#define IMU_PUBLISHER_TIMER 0.01

//Typically, motion processing algorithms should be run at a high rate, often around 200Hz,
//in order to provide accurate results with low latency. This is required even if the applica$
//updates at a much lower rate; for example, a low power user interface may update as slowly
//as 5Hz, but the motion processing should still run at 200Hz.
//Page 25 of MPU6050 datasheet.
//#define DEFAULT_SAMPLE_RATE_HZ  10

#define MPU_FRAMEID "base_imu"

//#include "AccelGyroSensorOffsets.h"

//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL
