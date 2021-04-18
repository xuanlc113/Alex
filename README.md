# CG1112 Group B01-1A

## Arduino

Upload Arduino.ino to Arduino Uno board

## Raspberry Pi

### TLS Server

```
cd Pi
make
./alex
```

### Hump Detector Client

run this after Hump Detector Server is running<br/>
humpDetector/cliffSensor/examples/client.cpp needs to be modified in line 30 to reference the ip address of Hump Detector Server (laptop ip address)

```
cd humpDetector/cliffSensor/build
cmake ..
make
./examples/client
```

### ROS

modify hosts and bashrc config file accordingly

```
cd catkin_ws
roslaunch rplidar_ros rplidar.launch
```

## Laptop

### TLS Client

```
cd Laptop
make
./laptop <ip address> 5000
```

### Hump Detector Server

```
cd humpDetector
gcc server.cpp -o server
./server
```

### Hector Slam

run after ros runs in Pi

```
cd catkin_ws
roslaunch rplidar_ros view_slam.launch
```
