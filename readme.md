
# Installation 
## ROS
You can find these installation instructions [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

    sudo apt update
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	sudo apt update
	sudo apt install -y ros-melodic-desktop-full
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
	sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
	sudo rosdep init
	rosdep update

## Ardupilot
### Installing Ardupilot and MAVProxy

    cd ~
    sudo apt install -y git
    git clone https://github.com/ArduPilot/ardupilot.git
    cd ardupilot
    git checkout Copter-3.6
    git submodule update --init --recursive
    sudo apt install -y python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk python-pip python-pexpect
    sudo pip install -y future pymavlink MAVProxy
    echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.bashrc
    echo 'export PATH=/usr/lib/ccache:$PATH' >> ~/.bashrc
    source ~/.bashrc
    cd ~/ardupilot/ArduCopter
    sim_vehicle.py -w

## Gazebo and Plugins
#### Gazebo

    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt update
    sudo apt install -y gazebo9 libgazebo9-dev
    sudo apt upgrade libignition-math2
    
#### Install Gazebo plugin for APM (ArduPilot Master) :

    cd ~
    git clone https://github.com/khancyr/ardupilot_gazebo.git
    cd ardupilot_gazebo
    git checkout dev
    mkdir build
    cd build
    cmake ..
    make -j4
    sudo make install
    echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
    echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc

#### Install Mavros

    sudo apt-get install -y ros-melodic-mavros
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    sudo chmod +x install_geographiclib_datasets.sh
    sudo ./install_geographiclib_datasets.sh

## Workspace Setup
### Creating Catkin Workspace

    mkdir catkin_ws
    cd catkin_ws
    mkdir -p src
    cd src
    catkin_init_workspace
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    cd ~/catkin_ws
    catkin_make
    source ~/catkin_ws/devel/setup.bash

### Compiling provided package
Download the provided zip file in your Downloads folder
```
cd ~/Downloads
unzip InterIIT_DRDO.zip
mv prius_description interiit22 prius_msgs ~/catkin_ws/src/
cd ~/catkin_ws
sed -i '8 a \ \ <build_depend>prius_msgs</build_depend>' src/interiit22/package.xml 
catkin_make
``` 

### Set Model Path

    echo 'export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/MP_DR_T18/models' >> ~/.bashrc

### Compiling our package
Extract our package in ~/catkin_ws/src/ and then run

    cd ~/catkin_ws
    catkin_make

### Install dependencies

    cd ~/catkin_ws/MP_DR_T18
    pip -r requirements.txt

## To the Run Simulation
Launch the world

    roslaunch MP_DR_T18 drdo_world1.launch

Start Ardupilot SITL

    sim_vehicle.py -v ArduCopter -f gazebo-iris --console

Once SITL provides the 3D fix run the autopilot

    rosrun MP_DR_T18 autopilot.py
