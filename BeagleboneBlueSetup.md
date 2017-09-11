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
* Install robotics package
```
sudo apt-get install roboticscape
```
* Install gpsd and other important software
```
sudo apt-get install samba gpsd gpsd-clients python3 python3-pip
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
