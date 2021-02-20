These instructions are for setting a new Beaglebone AI with Robotics Cape for Ladon Project development. 

# Initial Setup

* Download and burn the latest Beaglebone AI image using the instructions from [https://beagleboard.org/getting-started]
  * Use an IoT image with machine learning tools
* Connect and log in to the new board
* Edit /etc/network/interfaces and uncomment the connmanctl lines
* Uncomment the following lines:
```
connmanctl
connmanctl> tether wifi off
connmanctl> enable wifi
connmanctl> scan wifi
connmanctl> services
connmanctl> agent on
connmanctl> connect wifi_*_managed_psk
connmanctl> quit
```
* Add the following lines to enable the CAN interface:
```
auto can0
iface can0 inet manual
	pre-up /sbin/ip link set $IFACE type can bitrate 500000 
	up /sbin/ifconfig $IFACE up
	down /sbin/ifconfig $IFACE down
```

# Hardware Setup
Open ```/boot/uEnv.txt``` with a text editor and add the following lines:
```
uboot_overlay_addr0=/lib/firmware/BB-UART2-00A0.dtbo
uboot_overlay_addr1=/lib/firmware/BB-UART5-00A0.dtbo
```

# Expand partition to fill SD card

The installed partition is only 4 GB; it needs to be expanded to fill the entire SD card before we start installed more stuff.

* Open the disk for modification
```
sudo fdisk -u /dev/mmcblk0
```
* Type ```p``` to print the partition table. Note the ```Start``` point of the main partition and the number of bytes on the disk. 
* Type ```d``` to delete the old partition
* Type ```n``` to create a new partition starting at the same point as the original partition. Accept the default end point, which will include the whole disk.
* Type ```w``` to write the new partition and quit.
* Reboot to pick up new partition table
* Resize the filesystem to take advantage of the expanded partition
```
sudo resize2fs /dev/mmcblk0p1
```

* Update everything 
```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get dist-upgrade
```

# Software Installation

* Install gpsd and other important software
```
sudo apt-get install -y gpsd gpsd-clients cmake geographiclib-tools libgps-dev python3 
sudo apt-get install -y python3-pip python3-geographiclib libcurl4-gnutls-dev libproj-dev
sudo apt-get install -y libboost-all-dev gdb libgeographic-dev subversion xterm 
sudo apt-get install -y libfltk1.3-dev freeglut3-dev libpng-dev libjpeg-dev libxft-dev 
sudo apt-get install -y libxinerama-dev fluid libtiff5-dev librobotcontrol
```
# ROS Installation
Follow the source installation instructions for ROS Noetic (http://wiki.ros.org/noetic/Installation/Source)

Create workspace in ~/catkin_ws with the following instructions, after the installation of ROS:
- ``mkdir -p ~/catkin_ws/src``
- ``cd ~/catkin_ws``
- ``catkin_make``

## Additional ROS packages

Generally, you need to fetch these git repositories into your catkin_ws/src directory. Once they're all available, call ``catkin_make`` from ~/catkin_ws. 

* ``git clone https://github.com/SyllogismRXS/moos-ros-bridge.git``
* ``git clone https://github.com/ros/cmake_modules.git``
* ``git clone https://github.com/ros/class_loader.git``
* ``git clone https://github.com/ros/diagnostics.git``
* ``git clone https://github.com/ros/filters.git``
* ``git clone https://github.com/ros/pluginlib.git``
* ``git clone https://github.com/ros-industrial/ros_canopen.git``
* ``git clone https://github.com/ros-controls/ros_control.git``
* ``git clone https://github.com/ros/roslint.git``
* ``git clone https://github.com/ros-drivers/rosserial.git``

# Manual Library Installation

* Install rapidjson package
```
git clone https://github.com/Tencent/rapidjson.git
cd rapidjson
cmake . && make && sudo make install
```
* Install date package
```
git clone https://github.com/HowardHinnant/date.git
cd date/include
sudo cp -R date /usr/include
```
* Install eigen3
```
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
mkdir build
cd build
cmake .. && sudo make install
```
* Install autodiff
```
git clone https://github.com/autodiff/autodiff
cd autodiff
mkdir build
cd build
cmake .. && make && sudo make install
```
* Install nlohmann/json
```
git clone https://github.com/nlohmann/json.git
cd json/include
sudo cp -R nlohmann /usr/include
```
# Installing & Building MOOS-IvP

* Fetch the latest version of MOOS-IvP (update version number as needed)
```
svn co https://oceanai.mit.edu/svn/moos-ivp-aro/releases/moos-ivp-19.8.2 moos-ivp
```
* Build MOOS-IvP
```
cd moos-ivp
./build.sh
./build-check.sh
```
* There's a bug in the build-check.sh script that causes it to miss the build of proj. Additionally, gzaicview and vzaicview do not compile, so they too will show as failed. 
* Add /home/debian/moos-ivp/bin to your PATH variable in ~/.bashrc
* Note that if you want to run on the BBBlue from a remote desktop with Cygwin/X, you need to start the server with the +iglx option and do the following on both your Cygwin/X session and the Beaglebone:
```
export LIBGL_ALWAYS_INDIRECT=1
```
# Setting up GPSd

* Assuming you are using the standard Beaglebone Blue GPS port, add the entry ```/dev/ttyO2``` to the DEVICES list in ```/etc/default/gpsd```.
* Another entry will be added in the future for AIS
* Reboot the beaglebone or restart the gpsd process to pick up the change

# Installing magnetic model

* Use the helper script to fetch and install the magnetic models
```
geographiclib-get-magnetic all
```

# Other fixes
Some Beaglebone Blues have 'funky' bootloader setups. These may cause some peripherals, such as ADC and PRUs, not to work properly. You can check this by running:
```
sudo /opt/scripts/tools/version.sh
```
Check if there are multiple bootloader lines. If there are, run the following:
```
sudo dd if=/dev/zero of=/dev/mmcblk1 bs=1M count=10 
```
