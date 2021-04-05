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

void setupSerial() { Serial.begin(9600); }

int readSerial(char *buffer) {
    int count = 0;
    while (Serial.available()) {
        buffer[count++] = Serial.read();
    }

    return count;
}

void writeSerial(const char *buffer, int len) { Serial.write(buffer, len); }

/*
   Alex's motor drivers.

*/

ISR(TIMER0_COMPA_vect) {  // right fwd
    PORTD ^= 1 << 6;
}

ISR(TIMER0_COMPB_vect) {  // right bwd
    PORTD ^= 1 << 5;
}

ISR(TIMER2_COMPA_vect) {  // left fwd
    PORTB ^= 1 << 3;
}

ISR(TIMER2_COMPB_vect) {  // left bwd
    PORTB ^= 1 << 2;
}

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

    DDRB |= 0b00001100;
    DDRD |= 0b01100000;
    PORTB &= ~(1 << 3) & ~(1 << 2);
    PORTD &= ~(1 << 6) & ~(1 << 5);
    TCCR0A |= 0b10100001;
    TCNT0 = 0;
    OCR0A = 128;
    OCR0B = 128;
    TIMSK0 |= (1 << OCIE0A) | (1 << OCIE0B);

    TCCR2A |= 0b10100001;
    TCNT2 = 0;
    OCR2A = 128;
    OCR2B = 128;
    TIMSK2 |= (1 << OCIE0A) | (1 << OCIE0B);

    TCCR2B |= (1 << CS01) | (1 << CS00);
    TCCR0B |= (1 << CS01) | (1 << CS00);
}

void forward() {
    // analogWrite(LF, 100);
    // analogWrite(RF, 100);
    // analogWrite(LR, 0);
    // analogWrite(RR, 0);
    // delay(100);
    // stop();
    PORTD |= 1 << 6;
    PORTB |= 1 << 3;
    PORTD &= ~(1 << 5);
    PORTB &= ~(1 << 2);
}

void reverse() {
    // analogWrite(LR, 100);
    // analogWrite(RR, 100);
    // analogWrite(LF, 0);
    // analogWrite(RF, 0);
    // delay(100);
    // stop();
    PORTD |= 1 << 5;
    PORTB |= 1 << 2;
    PORTD &= ~(1 << 6);
    PORTB &= ~(1 << 3);
}

void left() {
    // analogWrite(LR, 100);
    // analogWrite(RF, 100);
    // analogWrite(LF, 0);
    // analogWrite(RR, 0);
    // delay(100);
    // stop();
    PORTD |= 1 << 5;
    PORTB |= 1 << 3;
    PORTD &= ~(1 << 6);
    PORTB &= ~(1 << 2);
}

void right() {
    // analogWrite(RR, 100);
    // analogWrite(LF, 100);
    // analogWrite(LR, 0);
    // analogWrite(RF, 0);
    // delay(100);
    // stop();
    PORTD |= 1 << 6;
    PORTB |= 1 << 2;
    PORTD &= ~(1 << 5);
    PORTB &= ~(1 << 3);
}

// Stop Alex. To replace with bare-metal code later.
void stop() {
    // analogWrite(LF, 0);
    // analogWrite(LR, 0);
    // analogWrite(RF, 0);
    // analogWrite(RR, 0);
    PORTB &= ~(1 << 3) & ~(1 << 2);
    PORTD &= ~(1 << 6) & ~(1 << 5);
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
    setupMotors();
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

    // TPacket recvPacket;  // This holds commands from the Pi

    // TResult result = readPacket(&recvPacket);

    // if (result == PACKET_OK)
    //     handlePacket(&recvPacket);
    // else if (result == PACKET_BAD) {
    //     sendBadPacket();
    // } else if (result == PACKET_CHECKSUM_BAD) {
    //     sendBadChecksum();
    // }

    TPacket recvPacket;  // This holds commands from the Pi

    if (receiveMsg) {
        char buffer[PACKET_SIZE];
        TResult result = deserialize(buffer, len, &recvPacket);
        if (result == PACKET_OK)
            handlePacket(&recvPacket);
        else if (result == PACKET_BAD) {
            sendBadPacket();
        } else if (result == PACKET_CHECKSUM_BAD) {
            sendBadChecksum();
        }
    }
}
