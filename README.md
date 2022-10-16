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

## :two: Navigation 실행

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

#### 2) Kobuki_node 실행

:computer: Desktop
```
$ 
```
























