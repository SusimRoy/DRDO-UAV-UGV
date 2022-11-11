# Inter IIT Tech Meet 10.0

# Installation 
## ROS
You can find these installation instructions [here](wiki.ros.org/melodic/Installation/Ubuntu).
#### Setup your sources.list
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#### Set up your keys
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
#### Update packages and install ROS
	sudo apt update
	sudo apt install ros-melodic-desktop-full
#### Setup the environment
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
	source ~/.bashrc	
#### Dependencies
	sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
#### Rosdep
	sudo apt install python-rosdep
	sudo rosdep init
	rosdep update

> Note : We recommend you to create a new catkin_ws since you have to install mavros from source which needs you to do `catkin build`.

## Ardupilot
### Installing Ardupilot and MAVProxy
#### Clone ArduPilot

In home directory:
```
cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout Copter-3.6
git submodule update --init --recursive
```

#### Install dependencies:
```
sudo apt install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk python-pip python-pexpect
```

#### Use pip (Python package installer) to install mavproxy:
```
sudo pip install future pymavlink MAVProxy
```

Open `~/.bashrc` for editing:
```
gedit ~/.bashrc
```

Add these lines to end of `~/.bashrc` (the file open in the text editor):
```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
```

Save and close the text editor.

Reload `~/.bashrc`:
```
. ~/.bashrc
```

Run SITL (Software In The Loop) once to set params:
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```
## Gazebo and Plugins
#### Gazebo

Setup your computer to accept software from http://packages.osrfoundation.org:
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:
```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Reload software list:
```
sudo apt update
```
Install Gazebo:
```
sudo apt install gazebo9 libgazebo9-dev
```
### Install Gazebo plugin for APM (ArduPilot Master) :
```
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
git checkout dev
```
build and install plugin
```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
```
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
```
Set paths for models:
```
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
. ~/.bashrc
```

#### Run Simulator
In one Terminal (Terminal 1), run Gazebo:
```
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```

In another Terminal (Terminal 2), run SITL:
```
cd ~/ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

### Install Mavros
please install mavros from source because we have changed the frame from `LOCAL_NED` to `BODY_NED`.
> If you decide to use the already existing catkin_ws then be sure to remove the `build` and `devel` folders and do `catkin build`.

Link to install from source : [Link](https://docs.px4.io/master/en/ros/mavros_installation.html)

### Modifying Mavros
After you install from source, please change the file `set_velocity.cpp` located at `catkin_ws/src/mavros/src/plugins/setpoint_velocity.cpp`.

New file : [Link](https://github.com/MP-DR-T13/setpoint_velocity_changed/blob/main/setpoint_velocity.cpp)

### Running Simulation
- Download the `interiit22` package from this repo and include it in your `catkin_ws`
- Do `catkin build` (assuming you have done `catkin build` while installing mavros)
- Go the the `glimbal_small_2d` folder of `ardupilot_gazebo/models` folder and change Line 164 in the `models.sdf` file.
```
<pointCloudCutoffMax>20.0</pointCloudCutoffMax>
```
We increased the range of Depth camera from 10m to 20m.

- cd to the folder containing simulation sh file
```
 cd catkin_ws/src/interiit22
 ```
 - make it executable 
 ```
 chmod u+x startsim.sh
 ```
- Open the terminal and launch the world file
```
./startsim.sh
```
- Now please wait until the MavProxy console shows that the GPS is correct and now open a new terminal and run the following command . This will takeoff the drone upto 15m height.
```
rosrun interiit22 arm_and_takeoff.py --connect 127.0.0.1:14550
```


- Now open a new terminal and run the `move_drone.py`
```
rosrun interiit22 move_drone.py
```
- Now you can open rviz and visualize the topic `/image_topic_2`. This is the main topic which draws the contours on the image
- Now start recording through rosbag
```
rosbag record -O bag_file_name /mavros/setpoint_velocity/cmd_vel_unstamped
```
- After the drone completely moves the world, press Ctrl+C to stop the reording
- Now play the recorded bag file 
```
rosbag play bag_file_name.bag
```
- Now open a new terminal and run
```
rosrun interiit22 move_prius.py
```
###Collaborators:
Devyani Gorkar
Susim Mukul Roy
Aaditya Baranwal
Rahul Gopathi
Sainath Reddy
Challa Bhavani Shankar
