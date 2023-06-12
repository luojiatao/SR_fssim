# How to Run It in your Workspace：
Install sudo apt install ros-melodic-desktop-full and sudo apt install python-catkin-tools
Clone this repository to an existing ROS Workspace initialized with catkin init
Run cd SR_fssim/src/fssim/ from home.
Run sudo bash Download_model.sh to download gazebo models or the default website will stop the simulation.
Run catkin build
Source the workspace
After successful building, run the simulator with roslaunch fssim auto_fssim.launch. RVIZ window will start. NOTE: You might need to untick and tick FSSIM Track and RobotModel in RVIZ in order to load the STL files. NOTE: This  [Wrn] [ModelDatabase.cc:339] Getting models from[http://gazebosim.org/models/]. This may take a few seconds. will stop the simulation. Check step.2, or disconnect the Internet and go on.
The terminal will inform you what is happening. The loading time takes around 20 seconds. When Sending RES GO show up in the terminal, you can start controlling the vehicle with /fssim/cmd topic.
(在catkin build过程中会出现缺失依赖等情况，根据终端反馈的信息补充相关依赖————sudo apt install ros-melodic-*)
# 功能包介绍
- fssim
  - 北理工开源的仿真平台
- fssim_gui
  - 键盘控制fssim赛车运行的qt小工具

## 使用方式

2. 将fssim和fssim_gui放在同一工作空间的src文件夹下

3. `catkin_make` 编译

4. 运行fssim
   ```
   source ./devel/setup.bash
   roslaunch fssim auto_fssim.launch
   ```
5. 运行fssim_gui
   ```
   source ./devel/setup.bash
   rosrun fssim_gui fssim_gui
   ```
## 配置说明

- 开启或关闭 激光雷达点云数据话题

  - 在 `fssim_description/cars/config` 文件夹下的 `sensors.yaml` 文件中，修改

  - ```
    sensors:
      lidar:
        enabled: false
        topic: /velodyne_points
        pos:
          x: 0.75
          y: 0.0
          z: 0.15
    ```

  - 中的enabled，true开启，false关闭
  
  - 修改过后，记得保存文件。
  
- 开启或关闭 gazebo可视化界面

  - 在 `fssim/launch` 文件夹下的 `fssim.launch` 文件中，修改

  - ```
    <!-- Gazebo -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="verbose" value="false"/>
        <arg name="world_name" value="$(arg world_name)"/>
        <arg name="gui" value="true"/>
        <arg name="debug" value="false"/>
    </include>
    ```

  - 中的 `<arg name="gui" value="true"/>`，true开启，false关闭

  - 修改过后，记得保存文件。



