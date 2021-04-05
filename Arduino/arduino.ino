#include <Adafruit_TCS34725.h>
#include <math.h>
#include <serialize.h>
#include <stdarg.h>

#include "colorsensor.h"
#include "constants.h"
#include "packet.h"

#define LF 6   // Left forward pin
#define LR 5   // Left reverse pin
#define RF 10  // Right forward pin
#define RR 11  // Right reverse pin

// Reads in data from the serial port and
// deserializes it.Returns deserialized
// data in "packet".
TResult readPacket(TPacket *packet) {
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if (len == 0)
        return PACKET_INCOMPLETE;
    else
        return deserialize(buffer, len, packet);
}

void sendStatus() {  // changed to send color
    TPacket statusPacket;
    statusPacket.packetType = PACKET_TYPE_RESPONSE;
    statusPacket.command = RESP_STATUS;
    statusPacket.params[0] = senseColor();
    sendResponse(&statusPacket);
}

void sendMessage(const char *message) {  // Sends text messages back to the Pi
    TPacket messagePacket;
    messagePacket.packetType = PACKET_TYPE_MESSAGE;
    strncpy(messagePacket.data, message, MAX_STR_LEN);
    sendResponse(&messagePacket);
}

void dbprint(char *format, ...) {
    va_list args;
    char buffer[128];
    va_start(args, format);
    vsprintf(buffer, format, args);
    sendMessage(buffer);
}

void sendBadPacket() {  // Tell the Pi that it sent us a packet with a bad magic number.
    TPacket badPacket;
    badPacket.packetType = PACKET_TYPE_ERROR;
    badPacket.command = RESP_BAD_PACKET;
    sendResponse(&badPacket);
}

void sendBadChecksum() {  // Tell the Pi that it sent us a packet with a bad checksum.

    TPacket badChecksum;
    badChecksum.packetType = PACKET_TYPE_ERROR;
    badChecksum.command = RESP_BAD_CHECKSUM;
    sendResponse(&badChecksum);
}

void sendBadCommand() {  // Tell the Pi that we don't understand its command sent to us.

    TPacket badCommand;
    badCommand.packetType = PACKET_TYPE_ERROR;
    badCommand.command = RESP_BAD_COMMAND;
    sendResponse(&badCommand);
}

void sendBadResponse() {
    TPacket badResponse;
    badResponse.packetType = PACKET_TYPE_ERROR;
    badResponse.command = RESP_BAD_RESPONSE;
    sendResponse(&badResponse);
}

void sendOK() {
    TPacket okPacket;
    okPacket.packetType = PACKET_TYPE_RESPONSE;
    okPacket.command = RESP_OK;
    sendResponse(&okPacket);
}

// Takes a packet, serializes it then sends it out over the serial port.
void sendResponse(TPacket *packet) {
    char buffer[PACKET_SIZE];
    int len;

    len = serialize(buffer, packet, sizeof(TPacket));
    writeSerial(buffer, len);
}

// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial() {
    Serial.begin(9600);

    // USCR0C = 0b00000110;
    // setBaud(57600);
    // UCSR0A = 0;
}

// void setBaud(int baudRate) {
//     int b = round(F_CPU / (16.0 * baudRate)) - 1;
//     UBRR0H = (b >> 8);
//     UBRR0L = b;
// }

void startSerial() {
    // UCSR0B = 0b10111000;
}

// volatile int receiveMsg = 0;
// volatile char buffer[PACKET_SIZE];

// ISR(USART_RX_VECT) { // markers allow for continuous sending
//     int isReceiving = 0;
//     char startMarker = '<';
//     char endMarker = '>';
//     int i = 0;

//     unsigned char data = UDR0;

//     if (data == startMarker) {
//         data = UDR0;
//         while (data != endMarker) {
//             buffer[i++] = data;
//             data = UDR0;
//         }
//         receiveMsg = 1;
//     }
// }

// ISR(USART_UDRE_VECT) {} // do we need this>

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.
int readSerial(char *buffer) {
    int count = 0;

    while (Serial.available()) buffer[count++] = Serial.read();

    return count;

    // for (int i = 0; i < 100; i++) {
    //     while ((USCR0A & 0b10000000) == 0)
    //         ;
    //     buffer[i] = UDR0;
    // }
}

// Write to the serial port. Replaced later with
// bare-metal code
void writeSerial(const char *buffer, int len) {
    Serial.write(buffer, len);

    // for (int i = 0; i < len; i++) {
    //     while ((USCR0A & 0b00100000) == 0)
    //         ;
    //     UDR0 = *buffer++;
    // }
}

/*
   Alex's motor drivers.

*/

void setupMotors() {
    //  DDRD |= 0b01100000;
    //  DDRB |= 0b00001100;
    //  TCNT0 = 0;
    //  TIMSK0 |= 0b110; // OCIEA = 1 OCIEB = 1
    //  OCR0A = 128;
    //  OCR0B = 128;
    //  TCCR0A = 0b10000001;
    //
    //  TCNT2 = 0;
    //  TIMSK2 |= 0b110; // OCIEA = 1 OCIEB = 1
    //  OCR2A = 128;
    //  OCR2B = 128;
    //  TCCR2A = 0b10000001;
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors() {
    //  TCCR0B |= 0b00000011;
    //  TCCR2B |= 0b00000011;
}

// Convert percentages to PWM values
int pwmVal(float speed) {
    if (speed < 0.0) speed = 0;

    if (speed > 100.0) speed = 100.0;

    return (int)((speed / 100.0) * 255.0);
}

void forward() {
    analogWrite(LF, 100);
    analogWrite(RF, 100);
    analogWrite(LR, 0);
    analogWrite(RR, 0);
    delay(100);
    stop();
}

void reverse() {
    analogWrite(LR, 100);
    analogWrite(RR, 100);
    analogWrite(LF, 0);
    analogWrite(RF, 0);
    delay(100);
    stop();
}

void left() {
    analogWrite(LR, 100);
    analogWrite(RF, 100);
    analogWrite(LF, 0);
    analogWrite(RR, 0);
    delay(100);
    stop();
}

void right() {
    analogWrite(RR, 100);
    analogWrite(LF, 100);
    analogWrite(LR, 0);
    analogWrite(RF, 0);
    delay(100);
    stop();
}

// Stop Alex. To replace with bare-metal code later.
void stop() {
    analogWrite(LF, 0);
    analogWrite(LR, 0);
    analogWrite(RF, 0);
    analogWrite(RR, 0);
}

/*
   Alex's setup and run codes

*/

void handleCommand(TPacket *command) {
    switch (command->command) {
        // For movement commands, pawwsram[0] = distance, param[1] = speed.
        case COMMAND_FORWARD:
            sendOK();
            forward();
            break;

        case COMMAND_REVERSE:
            sendOK();
            reverse();
            break;

        case COMMAND_TURN_LEFT:
            sendOK();
            left();
            break;

        case COMMAND_TURN_RIGHT:
            sendOK();
            right();
            break;

        case COMMAND_STOP:
            sendOK();
            stop();
            break;

        case COMMAND_GET_STATS:
            sendStatus();
            break;

        default:
            sendBadCommand();
    }
}

void waitForHello() {
    int exit = 0;

    while (!exit) {
        TPacket hello;
        TResult result;

        do {
            result = readPacket(&hello);
        } while (result == PACKET_INCOMPLETE);

        if (result == PACKET_OK) {
            if (hello.packetType == PACKET_TYPE_HELLO) {
                sendOK();
                exit = 1;
            } else
                sendBadResponse();
        } else if (result == PACKET_BAD) {
            sendBadPacket();
        } else if (result == PACKET_CHECKSUM_BAD)
            sendBadChecksum();
    }  // !exit
}

void setup() {
    cli();
    setupSerial();
    startSerial();
    setupMotors();
    startMotors();
    // setupColorSensor();
    sei();
}

void handlePacket(TPacket *packet) {
    switch (packet->packetType) {
        case PACKET_TYPE_COMMAND:
            handleCommand(packet);
            break;

        case PACKET_TYPE_RESPONSE:
            break;

        case PACKET_TYPE_ERROR:
            break;

        case PACKET_TYPE_MESSAGE:
            break;

        case PACKET_TYPE_HELLO:
            break;
    }
}

void loop() {
    // senseColor();

    TPacket recvPacket;  // This holds commands from the Pi

    TResult result = readPacket(&recvPacket);

    if (result == PACKET_OK)
        handlePacket(&recvPacket);
    else if (result == PACKET_BAD) {
        sendBadPacket();
    } else if (result == PACKET_CHECKSUM_BAD) {
        sendBadChecksum();
    }
}
