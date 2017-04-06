# Myo_Blink example

You can either run this in a docker container, or run it on a Ubuntu 16.04 with ROS kinetic installed.
**In any case the your machine or the docker HOST machine needs to have the ftdi drivers installed.**

## Dependencies
### Install the ftdi drivers

```
wget http://www.ftdichip.com/Drivers/D2XX/Linux/libftd2xx-x86_64-1.3.6.tgz
gunzip libftd2xx-x86_64-1.3.6.tgz
tar xvf libftd2xx-x86_64-1.3.6.tar
cp -r ./release/build/lib* /usr/local/lib
ln -s /usr/local/lib/libftd2xx.so.1.3.6 /usr/local/lib/libftd2xx.so
chmod 0755 /usr/local/lib/libftd2xx.so.1.3.6
cp ./release/ftd2xx.h  /usr/local/include
cp ./release/WinTypes.h  /usr/local/include
ldconfig -v|grep ftd2xx
```

### Create a udev file to automatically load the right ftd2xx driver
```
wget https://raw.githubusercontent.com/Roboy/myo_blink/master/30-ftdi.rules
sudo cp 30-ftdi.rules /etc/udev/rules.d/
```

### Starting inside a docker container
When you are doing it in docker use the following command after adjusting the paths to point to the repositories.
```
docker run --rm -ti --privileged -v /dev/bus/usb:/dev/bus/usb -v ~/Development/Roboy/myo_blink/:/cm_workspace/src/myo_blink -v ~/Development/Roboy/flexrayusbinterface/:/cm_workspace/src/flexrayusbinterface -v ~/Development/Roboy/common_utilities/:/cm_workspace/src/common_utilities  ros:kinetic /bin/bash
```

### Install system dependencies

Once you are inside the container or 

if you are running 16.04 and kinetic directly run through the following commands
```
apt-get update
apt-get install -y wget cmake
```

### Install the ros nodes
#### If you don't already have a catkin workspace, create one:
```
mkdir -p catkin_workspace/src
cd catkin_workspace/src
catkin_init_workspace
```

#### Download the repos
```
git clone https://github.com/Roboy/flexrayusbinterface.git
git clone https://github.com/Roboy/myo_blink.git
git clone https://github.com/Roboy/common_utilities.git

```

## Build it
```
cd .. # you should be in catkin_workspace
catkin_make
```

## Configure it
Update the serial number in the yaml config file in `.../myo_blink/config` to match the serial of your FTDI device.

i.e. line 2: https://github.com/Roboy/myo_blink/blob/master/config/myoarm_controllers.yaml

Adapt other parameters in the yaml file to your liking.

## Play with it

### Run it
```
source devel/setup.bash
roslaunch myo_blink myo_blink.launch
```

### Read the spring displacement sensor
```
rostopic echo /myo_blink/muscles/0/sensors/displacement -c
```

### Move the motor
Control mode (action):
- 'move to' - PositionController
- 'move with' - VelocityController
- 'keep' - Effort / ForceController
```
rosservice call /myo_blink/move "ganglion: 0
muscle: 1
action: 'move to'
setpoint: 10.0" 
```
