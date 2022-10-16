# Kobuki_navigation

<img
  src = "https://user-images.githubusercontent.com/94280596/196031105-8ef81310-e586-401d-b083-aa7039b1802c.png"
  width = "300"
  height = "300"
/>


<br><br><br><br>


## :bell: Setting
### OS : Unbuntu 20.0.4 LTS, Rapbian Buster
### Ros : Noetic Ninjemys
### Robot : Kobuki (Yujinrobot),  Raspberry Pi 4 (Model B)
### Sensor : 360 Laser Distance Sensor LDS-01 (Lidar_ROBOTIS)


<br><br><br><br><br><br><br>


## :one: Navigation을 위한 Ros Package

### :speech_balloon: 설명
- 로봇으로는 *kobuki*를 사용하고, 센서로는 *LDS-01*를 사용하였다.
- 네비게이션 실습 진행 시 SLAM 실습 때 사용한 *지도*를 사용할 예정이다. 

<br>

- **"kobuki_node", "hls_lfcd_lds_driver"** 런치파일은 **"Rapsbarry Pi"** 에서 진행한다.
- 위 두개를 제외한 모든 런치 파일의 실행은 **"데스크톱"**에서 진행한다.

<br>

#### :wrench: 패키지 설치
- **kobuki_tf**
- **navigation**
- **move_base**
- **amcl**
- **map_server**

<br><br><br><br>

## :two: Navigation 런치 파일 수정

#### :paperclip: 기존의 launch 파일을 실행 시 Error가 난다. 필요한 부분을 나의 로봇에 맞게 수정해야 한다.

<br><br>

### :blue_book: 원본 [ kobuki_navigation.launch ]
```
<launch>
  <!-- kobuki model -->
  <arg name="urdf_file"
    default="$(find xacro)/xacro.py '$(find kobuki_description)/urdf/kobuki_standalone.urdf.xacro'" />
  <param name="robot_description" command="$(arg urdf_file)" />
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen">
    <param name="publish_frequency" type="double" value="5.0" />
  </node>

  <!-- sensor -->
  <node pkg="urg_node" type="urg_node" name="kobuki_urg_node" output="screen">
    <param name="frame_id" value="base_scan" />
  </node>

  <!-- tf -->
  <node pkg="kobuki_tf" type="kobuki_tf" name="kobuki_tf" output="screen">
  </node>

  <!-- Map server -->
  <arg name="map_file" default="$(find kobuki_navigation)/maps/map.yaml"/>
  <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)">
  </node>

  <!-- AMCL -->
  <include file="$(find kobuki_navigation)/launch/amcl.launch.xml"/>

  <!-- move_base -->  
  <arg name="cmd_vel_topic" default="/mobile_base/commands/velocity" />
  <arg name="odom_topic" default="odom" />
  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <rosparam file="$(find kobuki_navigation)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find kobuki_navigation)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find kobuki_navigation)/param/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find kobuki_navigation)/param/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find kobuki_navigation)/param/base_local_planner_params.yaml" command="load" />
    <rosparam file="$(find kobuki_navigation)/param/move_base_params.yaml" command="load" />
    <remap from="cmd_vel" to="$(arg cmd_vel_topic)"/>
    <remap from="odom" to="$(arg odom_topic)"/>
  </node>

</launch>
```

<br><br><br>

### :pencil2: 수정 내용


#### 1) kobuki_model

- 기존의 xacro.py -> xacro 로 바꾼다. 
```
<arg name="urdf_file" default="$(find xacro)/xacro '$(find kobuki_description)/urdf/kobuki_standalone.urdf.xacro'"/>
```


#### 2) sensor
- 센서 값은 비워둔다. (라즈베리파이 에서 실행하기 때문)



#### 3) Map_server
- SLAM시 맵을 저장한 위치 파일로 경로를 바꿔준다.
- carto_map 디렉토리에 map.yaml을 가리킨다.
```
  <arg name="map_file" default="$(find kobuki_navigation)/carto_map/map.yaml"/>
```

<br><br><br><br>

### :green_book: 수정본
```
<launch>

  <!-- kobuki model -->
  <arg name="urdf_file" default="$(find xacro)/xacro '$(find kobuki_description)/urdf/kobuki_standalone.urdf.xacro'"/>
  <param name="robot_description" command="$(arg urdf_file)"/>
  
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen">
    <param name="publish_frequency" type="double" value="5.0" />
  </node>

  <!-- kobuki control option 
  <include file="$(find kobuki_navigation)/launch/velocity_smoother.launch.xml"/>
  <include file="$(find kobuki_navigation)/launch/safety_controller.launch.xml"/> 
  -->

  <!-- sensor -->



  <!-- tf -->
  <node pkg="kobuki_tf" type="kobuki_tf" name="kobuki_tf" output="screen">
  </node>

  <!-- Map server -->
  <arg name="map_file" default="$(find kobuki_navigation)/maps/map.yaml"/>
  <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)">
  </node>

  <!-- AMCL -->
  <include file="$(find kobuki_navigation)/launch/amcl.launch.xml"/>

  <!-- move_base -->  
  <arg name="cmd_vel_topic" default="/mobile_base/commands/velocity" />
  <arg name="odom_topic" default="odom" />
  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <rosparam file="$(find kobuki_navigation)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find kobuki_navigation)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find kobuki_navigation)/param/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find kobuki_navigation)/param/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find kobuki_navigation)/param/dwa_local_planner_params.yaml" command="load" />
    <rosparam file="$(find kobuki_navigation)/param/move_base_params.yaml" command="load" />
    <rosparam file="$(find kobuki_navigation)/param/base_local_planner_params.yaml" command="load" /> 
    <remap from="cmd_vel" to="$(arg cmd_vel_topic)"/>
    <remap from="odom" to="$(arg odom_topic)"/>
  </node>
</launch>
```



<br><br><br><br>

## :three: Navigation 실행

### :speech_balloon: 설명

#### 1) 소스 다운로드 및 컴파일

```
$ cd ~/kobuki_ws/src
```

```
$ git clone https://github.com/oroca/rosbook_kobuki.git
```

```
$ cd ~/kobuki_ws && catkin_make
```

<br><br>

#### 2) Kobuki_node 및 라이다 센서 실행

:computer: Desktop
```
$ roscore
```

<br>

:strawberry: Raspberry Pi
```
$ roslaunch kobuki_node minimal.launch
```

```
$ roslaunch hls_lfcd_lds_driver hlds_laser.launch
```

<br><br>

#### 3) Kobuki_navigation 실행

:computer: Desktop
```
$ roslaunch kobuki_navigation kobuki_navigation.launch
```







