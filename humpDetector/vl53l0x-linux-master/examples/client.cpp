// Client side C/C++ program to demonstrate Socket programming
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>

#include "VL53L0X.hpp"

#define THRESHOLD 500

#define PORT 8080

int main(int argc, char const *argv[]) {
    int sock = 0, valread;
    struct sockaddr_in serv_addr;
    char *hello = "HUMP!";
    char buffer[1024] = {0};
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Socket creation error \n");
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("\nConnection Failed \n");
        return -1;
    }

    VL53L0X sensor;
    sensor.initialize();
    sensor.setTimeout(200);

    int hasHump = 0;

    while (1) {
        uint16_t distance = sensor.readRangeSingleMillimeters();
        if (distance < THRESHOLD && hasHump == 0) {
            hasHump = 1;
            send(sock, hello, strlen(hello), 0);
            printf("Hello message sent\n");
            valread = read(sock, buffer, 1024);
            printf("%s\n", buffer);
        }
        if (distance > THRESHOLD) {
            hasHump = 0;
        }
    }
    return 0;
}