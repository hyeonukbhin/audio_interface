sHRI-Lab : Audio Interface
===========================================================

ROS 기반 Audio 입출력 Interface

## 1. Description

설명 수정

현재 stt_converter google v1beta1 문제가 있어서 동작 안함.
audio_stramer + audio_signal_processor 만 이용함. (수정 !!꼮!!해야함 190505)

### 1.1 Framework Structure

그림 수정
![H/W Structure](./assets/images/audio_interface.png)

### 2.1 Requirements

-	Ubuntu 16.04 or lator
-	[ROS](http://wiki.ros.org/) - Robot Operating System.
-	Many python package(requirements.txt에 기재)
```
sudo apt-get install libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0
sudo pip install -r requirements_py2.txt
sudo pip3 install -r requirements_py3.txt

sudo pip install --upgrade pip setuptools
sudo pip install --upgrade pip setuptools
sudo pip install --upgrade pyasn1
```

## 3. Usage
--------

```
roslaunch bringup_audio_interface bringup_audio_interface.launch
```
