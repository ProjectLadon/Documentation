These instructions are for setting a new Beaglebone Blue for Ladon Project development. 

# Initial Setup

* Download and burn the latest BBBLue image using the instructions from [https://beagleboard.org/getting-started]
* Connect and log in to the new board
* Edit /etc/network/interfaces and uncomment the connmanctl lines
* Type the following lines:

'''
connmanctl
connmanctl> tether wifi off
connmanctl> enable wifi
connmanctl> scan wifi
connmanctl> services
connmanctl> agent on
connmanctl> connect wifi_*_managed_psk
connmanctl> quit
'''

* Update everything 

'''
sudo apt-get update
sudo apt-get upgrade
'''

* Install robotics package
