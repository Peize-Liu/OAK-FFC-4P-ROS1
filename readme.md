# Info
OAK中国提供的ROS1 driver，本仓库只供HKUST-UAV 和 ZJU-FAST Lab 以及 杭州微分智飞使用，请勿扩散

# Installation
Depth AI SDK 安装
```shell
sudo wget -qO- https://raw.githubusercontent.com/luxonis/depthairos/
main/install_dependencies.sh | sudo bash
```
```shell
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

```shell
mkdir -p ~/oak_ws/src
# cp this rep into unde ~/oak_ws/src
cd ../
rosdep install --from-paths src --ignore-src -r -y
catkin_make_isolated
```

Launch
```shell
source devel_isolated/setup.bash
roslaunch ros1_oak_ffc_sync oak_ffc_sync.launch
```

