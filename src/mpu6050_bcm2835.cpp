#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_datatypes.h>

// i2c
#include "I2Cdev.h"

// mpu6050 DMP
#include "MPU6050_6Axis_MotionApps20.h"

#include "mpu6050_bcm2835_def.h"
#include "mpu6050_bcm2835_globals.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

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

std::string frame_id;

// mpu offsets
int ax, ay, az, gx, gy, gz;

bool debug = false;

// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
bool ado = false;

double angular_velocity_covariance, pitch_roll_covariance, yaw_covariance, linear_acceleration_covariance;
double linear_acceleration_stdev_, angular_velocity_stdev_, yaw_stdev_, pitch_roll_stdev_;

void mpu6050_device_reset() {
    mpu.reset();
}

int mpu6050_device_setup() {
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

}

//
// SETUP IMU PARAMETERS
//
void mpu6050_ros_params_setup(ros::NodeHandle pn) {

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

}

void mpu6050_ros_publishers_setup(ros::NodeHandle n) {

    imu_pub = n.advertise<sensor_msgs::Imu>("imu/data", 10);
    imu_euler_pub = n.advertise<geometry_msgs::Vector3Stamped>("imu/euler", 10);
    mag_pub = n.advertise<geometry_msgs::Vector3Stamped>("imu/mag", 10);
}

void mpu6050_process(ros::NodeHandle pn, ros::NodeHandle n) {

//    if(ros::Time::now().toSec() > imuPublisherTimer) {
        //
        // PROCESS IMU
        //

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


        // http://www.i2cdevlib.com/forums/topic/4-understanding-raw-values-of-accelerometer-and-$
            //The output scale for any setting is [-32768, +32767] for each of the six axes.
        //The default setting in the I2Cdevlib class is +/- 2g for the accel and +/- 250 deg/sec
        //for the gyro. If the device is perfectly level and not moving, then:
            //
            //    X/Y accel axes should read 0
            //    Z accel axis should read 1g, which is +16384 at a sensitivity of 2g
            //    X/Y/Z gyro axes should read 0
            //
            //In reality, the accel axes won't read exactly 0 since it is difficult to be perfect$
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

        //imuPublisherTimer = ros::Time::now().toSec() + IMU_PUBLISHER_TIMER;
//    }

}
