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

### :pushpin: 런치 파일 세부 설명
---
#### 1) kobuki model
- kobuki_description 패키지에서 kobuki_stanalone.urdf 의 로봇 3D 모델을 불러오고, 
  robot_state_publisher 를 통해 조인트 정보와 같은 **"로봇 상태"**를 상대 위치 변환인 tf로 발행하게 된다.
  이 과정이 있기에 Rviz 에서 로봇의 3차원 모델을 볼 수 있게 된다.
  
<br>

#### 2) sensor
- 거북이에 장착되어 있는 센서를 구동하기 위한 구문으로 **"frame_id"** 파라미터를 **"base_scan"** 으로 변환시켜 실행시키도록 한다.
  
<br>


#### 3) tf
- 오도메트리부터 센서 위치까지의 상태 위치 변환 **( odom -> base_footprint -> base_link -> base_scan )**
  정보를 **"tf"** 형태로 발행하는 kobuki_tf를 함께 실행해야 한다.

<br>


#### 4) Map_server
- kobuki_navigation/maps/floder에 저장된 지도 정보 **(map.yaml)** 를 
  불러와서 **map_server** 노드에 의해 토픽 형태로 지도가 발행된다.

<br>


#### 5) AMCL (Adaptive)
- AMCL 관련하여 acml 노드를 실행시키며, 관련 파라미터를 설정한다.

<br>


#### 6) moev_base
- 모션 계획에 필요한 **"costmap"** 관련 파라미터와 로봇에게 이동 속도 명령을 넘겨주는 **"base_local_planner"** 
  에 대한 설정 파라미터, 모션 계획을 총괄적으로 담당하는 move_base의 파라미터를 설정한다. 


<br>

---

<br><br><br>

### :pencil2: 수정 내용


#### 1) kobuki_model

- 기존의 xacro.py -> xacro 로 바꾼다. 
```
<arg name="urdf_file" default="$(find xacro)/xacro '$(find kobuki_description)/urdf/kobuki_standalone.urdf.xacro'"/>
```

<br>

#### 2) sensor
- 센서 값은 비워둔다. (라즈베리파이 에서 실행하기 때문)

<br>

#### 3) Map_server
- SLAM시 맵을 저장한 위치 파일로 경로를 바꿔준다.
- carto_map 디렉토리에 map.yaml을 가리킨다.
```
  <arg name="map_file" default="$(find kobuki_navigation)/carto_map/map.yaml"/>
```

<br><br>

### :green_book: [ kobuki_navigation.launch ] 수정본
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

<br><br>

### :speaker: param 수정
#### Ros 버전이 **melodic** or **noetic** 일 때 옛날버전과 실행 방법이 달라 추가 변경이 필요하다.
```
$ cd ~/kobuki_ws/src/rosbook_kobuki/kobuki_navigation/param
```

<br>

#### :memo:  local_costmap_params.yaml 
#### :memo:  global_costmap_params.yaml  
- global_frame을 **"/map"** -> **"map"** 으로 바꿔준다.

---


<br><br><br><br><br><br>

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
- 복수의 런치 파일로 구성되어 있어야 한다.
- 기본적으로 robot_model, sensor, tf, Map_server, amcl, move_base 등으로 구성되어 있다.

:computer: Desktop
```
$ roslaunch kobuki_navigation kobuki_navigation.launch
```


<br><br>

#### 4) RViz 실행
- 내비게이션에서 목적지 지정 명령 및 그 결과를 눈으로 확인 할 수 있도록 ROS의 시각화 툴 **"Rviz"** 를 구동한다.

:computer: Desktop
```
$ rosrun rviz rviz -d `rospack find kobuki_navigation`/rviz/kobuki_nav.rviz
```



<br><br>

#### 5) 초기 위치 추정
- 첫번째로 **"로봇의 초기 위치 추정 작업"** 을 해야 한다. Rviz 상단의 메뉴바 중 **"2D Pose Estimate"** 를 누르면
  녹색 화살표가 마우스 포인트로 나오며 이를 실제 로봇의 대략적인 위치에 놓고 정면방향을 화살표 방향과 맞게 한 후 클릭한다.
  
- 이것은 초기에 대략적인 로봇의 위치를 추정하기 위한 일종의 명령어이다. 이 과정을 거치면 로봇의 위치(x, y) 와 방향(θ)을 가지고
  스스로 자신의 위치를 추정하여(x, y, θ)가 세팅이 된다. 



<br><br>

#### 6) 목적지 설정 및 로봇 이동
- 초기 위치 설정을 마친 뒤, Rviz 상단의 메뉴바 중 **"2D Nav Goal"** 를 누르면 위와 마찬가지로 분홍색 화살표가 마우스 포인트로
  나온다. 이를 로봇이 가야하는 목적지에 클릭하고, 드래그 하여 방향을 설정하면 된다. 그럼 로봇이 목적지 까지 장애물을 피하여 이동할 것이다.

---


<br><br><br>

### :smile: 실행 화면

<br><br>

#### :camera: Rviz 화면
#### YOUTUBE : https://youtu.be/9GqPcyeN4jQ
https://user-images.githubusercontent.com/94280596/196049485-447e2f26-0bee-447a-8df4-e67ba4d139e6.mp4


<br><br>

#### :robot: 실제 화면
#### YOUTUBE : https://youtu.be/Cn7BN1utXQo
https://user-images.githubusercontent.com/94280596/196050830-876b4808-b9ac-4a17-89f1-f221c536ba0b.mp4







