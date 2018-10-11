sHRI-Lab : Audio Interface for DTC Project
===========================================================

ROS 기반 Audio 입출력 Interface

## 1. Description


### 1.1 Framework Structure

![H/W Structure](./assets/images/audio_interface.png)

### 2.1 Requirements

-	Ubuntu 14.04 or Ubuntu 16.04
-	[ROS](http://wiki.ros.org/) - Robot Operating System.
-	Many python package(requirements.txt에 기재)
```
sudo pip install -r requirements.txt
sudo apt-get install libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0
```

## 3. Usage
--------

```
roslaunch bringup_audio_interface bringup_audio_interface.launch
```
