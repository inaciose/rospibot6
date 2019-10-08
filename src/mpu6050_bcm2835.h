void mpu6050_device_reset();
int mpu6050_device_setup();
void mpu6050_ros_params_setup(ros::NodeHandle pn);
void mpu6050_ros_publishers_setup(ros::NodeHandle n);
void mpu6050_process(ros::NodeHandle pn, ros::NodeHandle n);
