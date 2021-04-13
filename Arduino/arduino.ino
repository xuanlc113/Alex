//#include <NewPing.h>
#include <serialize.h>

#include "colorsensor.h"
#include "constants.h"
#include "packet.h"

#define MAX_DISTANCE 300
#define sonarfstop 130
#define sonarbstop 180
#define sonarffaststop 200

#define DZ 33

#define echob A0
#define triggerb A1
unsigned long mikrosb, lastEchob = 0UL;
unsigned long echobdist;

#define echof A4
#define triggerf A5
unsigned long mikrosf, lastEchof = 0UL;
unsigned long echofdist;

enum Direction { FORWARD, REVERSE, LEFT, RIGHT, STOP };

Direction currentDirection;

int isFast = 0;

/*Alex Communication Routines.*/

TResult readPacket(TPacket *packet) {
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if (len == 0)
        return PACKET_INCOMPLETE;
    else
        return deserialize(buffer, len, packet);
}

void sendStatus() {
    TPacket statusPacket;
    statusPacket.packetType = PACKET_TYPE_RESPONSE;
    statusPacket.command = RESP_STATUS;
    statusPacket.params[0] = senseColor();
    sendResponse(&statusPacket);
}

void sendMessage(const char *message) {
    TPacket messagePacket;
    messagePacket.packetType = PACKET_TYPE_MESSAGE;
    strncpy(messagePacket.data, message, MAX_STR_LEN);
    sendResponse(&messagePacket);
}

void sendBadPacket() {
    TPacket badPacket;
    badPacket.packetType = PACKET_TYPE_ERROR;
    badPacket.command = RESP_BAD_PACKET;
    sendResponse(&badPacket);
}

void sendBadChecksum() {
    TPacket badChecksum;
    badChecksum.packetType = PACKET_TYPE_ERROR;
    badChecksum.command = RESP_BAD_CHECKSUM;
    sendResponse(&badChecksum);
}

void sendBadCommand() {
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

void sendResponse(TPacket *packet) {
    char buffer[PACKET_SIZE];
    int len;

    len = serialize(buffer, packet, sizeof(TPacket));
    writeSerial(buffer, len);
}

void setupSerial() { Serial.begin(9600); }

int readSerial(char *buffer) {
    int count = 0;

    while (Serial.available()) buffer[count++] = Serial.read();

    return count;
}

void writeSerial(const char *buffer, int len) { Serial.write(buffer, len); }

/*

    Alex's motor drivers.

*/

ISR(TIMER0_COMPA_vect) {}

ISR(TIMER0_COMPB_vect) {}

ISR(TIMER2_COMPA_vect) {}

ISR(TIMER2_COMPB_vect) {}

// right: 5, 6, TIMER 0
// left: 3, 11, TIMER 2
void setupMotors() {
    DDRB |= 0b00001000;
    DDRD |= 0b01101000;

    TCNT0 = 0;
    TIMSK0 |= 0b110;
    TCCR0A |= 0b1;
    OCR0A = 180;
    OCR0B = 180;
    TCCR0B = 0b00000011;

    TCNT2 = 0;
    TIMSK2 |= 0b110;
    TCCR2A |= 0b1;
    OCR2A = 200;
    OCR2B = 200;
    TCCR2B = 0b00000011;
}

void forward() {
    if (echofdist >= sonarfstop) {
        if (isFast) {
            OCR0A = 255;
            OCR0B = 255;
            OCR2A = 250;
            OCR2B = 250;
        } else {
            OCR0A = 175;
            OCR0B = 175;
            OCR2A = 180;
            OCR2B = 180;
        }
        TCCR0A = 0b00100001;
        TCCR2A = 0b10000001;
        currentDirection = FORWARD;
    }
}

void reverse() {
    if (echobdist >= sonarbstop) {
        OCR0A = 175;
        OCR0B = 175;
        OCR2A = 180;
        OCR2B = 180;
        TCCR0A = 0b10000001;
        TCCR2A = 0b00100001;
        currentDirection = REVERSE;
    }
}

void left() {
    OCR0A = 120;
    OCR0B = 120;
    OCR2A = 140;
    OCR2B = 140;
    TCCR0A = 0b10000001;
    TCCR2A = 0b10000001;
    currentDirection = LEFT;
}

void right() {
    OCR0A = 120;
    OCR0B = 120;
    OCR2A = 140;
    OCR2B = 140;
    TCCR0A = 0b00100001;
    TCCR2A = 0b00100001;
    currentDirection = RIGHT;
}

void stop() {
    TCCR0A = 0b00000001;
    TCCR2A = 0b00000001;
    currentDirection = STOP;
}

void toggleSpeed() {
    if (isFast) {
        isFast = 0;
        sendMessage("Slow Mode");
    } else {
        isFast = 1;
        sendMessage("Fast Mode");
    }
}

/*

    Alex's setup and run codes

*/

void handleCommand(TPacket *command) {
    switch (command->command) {
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
            sendOK();
            sendStatus();
            break;

        case COMMAND_TOGGLE_SPEED:
            sendOK();
            toggleSpeed();
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
    }
}

void setup() {
    cli();
    setupSerial();
    setupMotors();
    setupColorSensor();
    sei();
    digitalWrite(echob, LOW);
    pinMode(echob, INPUT);
    digitalWrite(triggerb, LOW);
    pinMode(triggerb, OUTPUT);
    digitalWrite(echof, LOW);
    pinMode(echof, INPUT);
    digitalWrite(triggerf, LOW);
    pinMode(triggerf, OUTPUT);
    Serial.begin(9600);
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

void dontBump() {
    if (isFast) {
        if (echofdist <= sonarffaststop && currentDirection == FORWARD) {
            stop();
        }
    } else {
        if (echofdist <= sonarfstop && currentDirection == FORWARD) {
            stop();
        }
    }
    if (echobdist <= sonarbstop && currentDirection == REVERSE) {
        stop();
    }
}

void sonarf() {
    digitalWrite(triggerf, HIGH);
    //  delayMicroseconds( 20 );
    digitalWrite(triggerf, LOW);

    while (!(digitalRead(echof)))
        ;

    mikrosf = micros();

    //  Test distance = (high level time * sound velocity (340M/S) / 2)
    echofdist = 0UL;
    while ((digitalRead(echof))) {
        echofdist++;
        if (echofdist > 1000000UL) {
            echofdist = 0UL;
            break;
        }
    }

    mikrosf = micros() - mikrosf;

    if (echofdist) {
        if ((mikrosf - lastEchof > DZ) && (lastEchof - mikrosf > DZ)) {
        }
        lastEchof = mikrosf;
    } else {
        lastEchof = 0UL;
    }
    delay(10);  // this seems to help settle readings
}

void sonarb() {
    digitalWrite(triggerb, HIGH);
    //  delayMicroseconds( 20 );
    digitalWrite(triggerb, LOW);

    while (!(digitalRead(echob)))
        ;

    mikrosb = micros();

    //  Test distance = (high level time * sound velocity (340M/S) / 2)
    echobdist = 0UL;
    while ((digitalRead(echob))) {
        echobdist++;
        if (echobdist > 1000000UL) {
            echobdist = 0UL;
            break;
        }
    }

    mikrosb = micros() - mikrosb;

    if (echobdist) {
        if ((mikrosb - lastEchob > DZ) && (lastEchob - mikrosb > DZ)) {
        }
        lastEchob = mikrosb;
    } else {
        lastEchob = 0UL;
    }
    delay(10);  // this seems to help settle readings
}

void loop() {
    dontBump();
    sonarf();
    sonarb();

    TPacket recvPacket;
    TResult result = readPacket(&recvPacket);

    if (result == PACKET_OK)
        handlePacket(&recvPacket);
    else if (result == PACKET_BAD) {
        sendBadPacket();
    } else if (result == PACKET_CHECKSUM_BAD) {
        sendBadChecksum();
    }
}
