These instructions are for setting a new Beaglebone Blue for Ladon Project development. 

# Initial Setup

* Download and burn the latest BBBLue image using the instructions from [https://beagleboard.org/getting-started]
  * An LXQT image (as opposed to an IoT image) enables the use of MOOS GUI applications
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
* Update everything 
```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get dist-upgrade
```

# Hardware Setup
Open ```/boot/uEnv.txt``` with a text editor and add the following lines:
```
uboot_overlay_addr0=/lib/firmware/BB-UART2-00A0.dtbo
uboot_overlay_addr1=/lib/firmware/BB-UART5-00A0.dtbo
uboot_overlay_addr2=/lib/firmware/BB-UART0-00A0.dtbo
uboot_overlay_addr3=/lib/firmware/BB-UART1-00A0.dtbo
uboot_overlay_addr4=/lib/firmware/BB-UART3-00A0.dtbo
uboot_overlay_addr5=/lib/firmware/BB-UART4-00A0.dtbo
```
Make sure that the following line is uncommented (and all other instances of the same command are commented out):
```
uboot_overlay_pru=/lib/firmware/AM335X-PRU-UIO-00A0.dtbo
```

# Expand partition to fill SD card

The installed partition is only 4 GB; it needs to be expanded to fill the entire SD card before we start installed more stuff. Luckily, there's a script:
```
sudo /opt/scripts/tools/grow_partition.sh
```

# Add swap 
https://paulbupejr.com/adding-swap-memory-to-the-beaglebone-black/

# Software Installation 

First set up a new SSH key with the instructions here: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent

Then clone the two repositories:
```
git clone git@github.com:ProjectLadon/ardupilot.git ardupilot-ladon --recurse-submodules
git clone git@github.com:ProjectLadon/Ladon-APM.git
```

Install a required python library:
```
sudo apt-get install python-future -y
```

Follow the setup instructions in https://github.com/ProjectLadon/Ladon-APM

Finally, build ardupilot with the following commands:
```
cd ~/ardupilot-ladon
./waf configure --board=blue
./waf build
```
Note that this takes several hours to complete. It will fail if the swap has not been created. 

# Other fixes
Some Beaglebone Blues have 'funky' bootloader setups. These may cause some peripherals, such as ADC and PRUs, not to work properly. You can check this by running:
```
sudo /opt/scripts/tools/version.sh
```
Check if there are multiple bootloader lines. If there are, run the following:
```
sudo dd if=/dev/zero of=/dev/mmcblk1 bs=1M count=10 
```
