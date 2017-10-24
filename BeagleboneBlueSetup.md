These instructions are for setting a new Beaglebone Blue for Ladon Project development. 

# Initial Setup

* Download and burn the latest BBBLue image using the instructions from [https://beagleboard.org/getting-started]
* Connect and log in to the new board
* Edit /etc/network/interfaces and uncomment the connmanctl lines
* Type the following lines:
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
* Update everything 
```
sudo apt-get update
sudo apt-get upgrade
```

# Expand partition to fill SD card

The installed partition is only 4 GB; it needs to be expanded to fill the entire SD card before we start installed more stuff.

* Open the disk for modification
```
sudo fdisk -u /dev/mmcblk0
```
* Type ```p``` to print the partition table. Note the ```Start``` point of the main partition and the number of bytes on the disk. 
* Type ```n``` to create a new partition starting at the same point as the original partition. Accept the default end point, which will include the whole disk.
* Type ```w``` to write the new partition and quit.
* Reboot to pick up new partition table
* Resize the filesystem to take advantage of the expanded partition
```
sudo resize2fs /dev/mmcblk0p1
```

# Software Installation

* Install robotics package:
```
sudo apt-get install roboticscape
```
* Install gpsd and other important software
```
sudo apt-get install -y samba gpsd gpsd-clients cmake geographiclib-tools libgps-dev
sudo apt-get install -y python3 python3-pip python3-geographiclib man-db
sudo apt-get install -y libboost-all-dev gdb libgeographic-dev
```
* If you are planning to install MOOS, install these as well:
```
sudo apt-get install -y subversion xterm libfltk1.3-dev freeglut3-dev libpng-dev libjpeg-dev libxft-dev libxinerama-dev fluid libtiff5-dev libproj-dev 
```

# SAMBA Setup

* Edit /etc/samba/smb.conf and set ```read only = no``` in the ```[homes]``` section
* Set the samba password for the default account
```
sudo smbpasswd -a debian
```
* Restart the samba server
```
sudo /etc/init.d/samba restart
```

# Manual Library Installation

* Install rapidjson package
```
git clone https://github.com/Tencent/rapidjson.git
cd rapidjson
cmake .
make; sudo make install
```
* Install date package
```
git clone https://github.com/HowardHinnant/date.git
cd date/include
sudo cp -R date /usr/include
```

# Installing & Building MOOS-IvP

* Fetch the latest version of MOOS-Ivp
```
svn co https://oceanai.mit.edu/svn/moos-ivp-aro/releases/moos-ivp-17.7 moos-ivp
```
* Build the software
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

* Assuming you are using the standard Beaglebone Blue GPS port, add the entry ```/dev/ttyO2``` to the DEVICES list.
* Another entry will be added in the future for AIS
* Reboot the beaglebone or restart the gpsd process to pick up the change

# Installing magnetic model

* Use the helper script to fetch and install the magnetic models
```
geographiclib-get-magnetic all
```
