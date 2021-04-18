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

Place hosts file in Pi root directory<br/>
modify hosts and bashrc config file accordingly

1. Modify bashrc:

```
sudo nano ~/.bashrc
```

2. Modify hosts file:

```
sudo nano /etc/hosts
```

3. Restart terminal

4. catkin_make repo:

```
cd ros_pi
catkin_make
```

5. run view_slam.launch:

```
roslaunch rplidar_ros view_slam.launch
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

Place hosts and bashrc config file in Linux root directory<br/>
run after ros runs in Pi

1. Modify bashrc:

```
sudo nano ~/.bashrc
```

2. Modify hosts file:

```
sudo nano /etc/hosts
```

3. Restart terminal

4. catkin_make repo:

```
cd ros_laptop
catkin_make
```

5. run view_slam.launch:

```
roslaunch rplidar_ros view_slam.launch
```
