# rospibot6
Robot ROS - STM32F, Pi, MPU6050, XV-11

STM32duino HAL library
https://github.com/rogerclarkmelbourne/Arduino_STM32

Raspberry Pi Libraries
https://github.com/jrowberg/i2cdevlib
http://www.airspayce.com/mikem/bcm2835/index.html

ROS packages
sudo apt-get install ros-kinetic-xv-11-laser-driver
#rosrun xv_11_laser_driver neato_laser_publisher _port:=/dev/tty1

sudo apt-get install ros-kinetic-gmapping
#rosrun gmapping slam_gmapping scan:=scan

sudo bash -c "source /opt/ros/kinetic/setup.bash; source /home/ubuntu/catkin_ws/devel/setup.bash; roslaunch rospibot6 robot.launch"
rostopic pub --once mcu_cmd std_msgs/Int16 "data: 3"
roslaunch rospibot6 teleop.launch


INSTALL I2Cdevlib:

sudo mkdir -p /usr/share/arduino/libraries
cd /usr/share/arduino/libraries
sudo git clone https://github.com/chrisspen/i2cdevlib.git

INSTALL Bcm2835:

cd /tmp
wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.50.tar.gz
tar zxvf bcm2835-1.50.tar.gz
cd bcm2835-1.50
./configure
make
make check
sudo make install
