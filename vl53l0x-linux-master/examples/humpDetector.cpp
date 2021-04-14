#include <iostream>

#include "VL53L0X.hpp"

#define THRESHOLD 500

int main() {
    VL53L0X sensor;
    sensor.initialize();
    sensor.setTimeout(200);

    int hasHump = 0;

    while (true) {
        uint16_t distance = sensor.readRangeSingleMillimeters();
        if (distance < THRESHOLD && hasHump == 0) {
            std::cout << "HUMP!" << std::endl;
            hasHump = 1;
        }
        if (distance > THRESHOLD) {
            hasHump = 0;
        }
    }
    // if (sensor.timeoutOccurred()) {
    // 	std::cout << "timeout!" << std::endl;
    // } else {
    // 	std::cout << distance << std::endl;
    // }

    return 0;
}
