# 功能包介绍
- fssim
  - 北理工开源的仿真平台
- fssim_gui
  - 自己开发的键盘控制fssim赛车运行的qt小工具



# 改进后的fssim
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



