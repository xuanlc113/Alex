# CG1112 Group B01-1A

How to use:

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

run this after Hump Detector Server is running.
humpDetector/cliffSensor/examples/client.cpp needs to be modified in line 30 to reference the ip address of Hump Detector Server (laptop ip address)

```
cd humpDetector/cliffSensor/build
cmake ..
make
./examples/client
```

### ROS

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
