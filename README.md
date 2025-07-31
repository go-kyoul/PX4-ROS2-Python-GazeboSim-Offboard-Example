# PX4-ROS2-offboard-example

https://docs.px4.io/main/en/ros2/user_guide.html

## 1. Install ROS2 Humble
```
pip install --user -U empy pyros-genmsg setuptools
```

## 2. Install PX4-Autopilot
```
cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl gz_x500
```

```
mkdir -p ~/ws_offboard_control/src/
cd ~/ws_offboard_control/src/
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
```

```
cd ..
source /opt/ros/humble/setup.bash
colcon build
```

```
source ~/ws_offboard_control/install/local_setup.bash
```


## 3. Setup Micro XRCE-DDS Agent & Client
```
cd
pip3 install --user -U empy pyros-genmsg setuptools
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

## 4. Sim
```
MicroXRCEAgent udp4 -p 8888
```

```
cd ~/PX4-Autopilot
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -s "iris:3"
```

```
python3 offboard_simple.py
```
