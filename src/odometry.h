void read_i2c_encoders();
void check_encoders_position();
void odometry_setup();
void odometry_publish(ros::NodeHandle pn, ros::NodeHandle n, ros::Publisher odom_pub, tf::TransformBroadcaster odom_broadcaster);
