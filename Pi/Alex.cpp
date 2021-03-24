#include "server_lib/constants.h"
#include "server_lib/make_tls_server.h"
#include "server_lib/netconstants.h"
#include "server_lib/packet.h"
#include "server_lib/serial.h"
#include "server_lib/serialize.h"
#include "server_lib/tls_common_lib.h"

// Arduino
#define PORT_NAME "/dev/ttyACM0"
#define BAUD_RATE B57600

// TLS Port Number
#define SERVER_PORT 5000

// CA Keys
#define KEY_FNAME "pikey/alex.key"
#define CERT_FNAME "pikey/alex.crt"
#define CA_CERT_FNAME "cert/signing.pem"
#define CLIENT_NAME "www.control.com"

// TLS Buffer
#define BUF_LEN 129

// TLS network active flag
static volatile int networkActive;

// This variable is used by sendNetworkData to send back responses
// to the TLS connection.  It is sent by handleNetworkData

static void *tls_conn = NULL;

/*
        Alex Serial Routines to the Arduino
        */

// Prototype for sendNetworkData
void sendNetworkData(const char *, int);

void handleErrorResponse(TPacket *packet) {
    printf("UART ERROR: %d\n", packet->command);
    char buffer[2];
    buffer[0] = NET_ERROR_PACKET;
    buffer[1] = packet->command;
    sendNetworkData(buffer, sizeof(buffer));
}

void handleMessage(TPacket *packet) {
    char data[33];
    printf("UART MESSAGE PACKET: %s\n", packet->data);
    data[0] = NET_MESSAGE_PACKET;
    memcpy(&data[1], packet->data, sizeof(packet->data));
    sendNetworkData(data, sizeof(data));
}

void handleStatus(TPacket *packet) {
    char data[65];
    printf("UART STATUS PACKET\n");
    data[0] = NET_STATUS_PACKET;
    memcpy(&data[1], packet->params, sizeof(packet->params));
    sendNetworkData(data, sizeof(data));
}

void handleResponse(TPacket *packet) {
    // The response code is stored in command
    switch (packet->command) {
        case RESP_OK:
            char resp[2];
            printf("Command OK\n");
            resp[0] = NET_ERROR_PACKET;
            resp[1] = RESP_OK;
            sendNetworkData(resp, sizeof(resp));
            break;

        case RESP_STATUS:
            handleStatus(packet);
            break;

        default:
            printf("Boo\n");
    }
}

void handleUARTPacket(TPacket *packet) {
    switch (packet->packetType) {
            // case PACKET_TYPE_COMMAND:
            //     // Only we send command packets, so ignore
            //     break;

        case PACKET_TYPE_RESPONSE:  // handshake
            handleResponse(packet);
            break;

        case PACKET_TYPE_ERROR:
            handleErrorResponse(packet);
            break;

        case PACKET_TYPE_MESSAGE:  // scan
            handleMessage(packet);
            break;
    }
}

void uartSendPacket(TPacket *packet) {
    char buffer[PACKET_SIZE];
    int len = serialize(buffer, packet, sizeof(TPacket));

    serialWrite(buffer, len);
}

void handleError(TResult error) {
    switch (error) {
        case PACKET_BAD:
            printf("ERROR: Bad Magic Number\n");
            break;

        case PACKET_CHECKSUM_BAD:
            printf("ERROR: Bad checksum\n");
            break;

        default:
            printf("ERROR: UNKNOWN ERROR\n");
    }
}

/*
        Alex Network Routines
        */

void sendNetworkData(const char *data, int len) {
    // Send only if network is active
    if (networkActive) {
        // Use this to store the number of bytes actually written to the TLS
        // connection.
        int c;

        // printf("WRITING TO CLIENT\n");

        if (tls_conn != NULL) {
            printf("WRITING TO CLIENT\n");
            /* TODO: Implement SSL write here to write data to the network. Note
              that handleNetworkData should already have set tls_conn to point
              to the TLS connection we want to write to. */
            sslWrite(tls_conn, data, len);
            /* END TODO */
        }

        // Network is still active if we can write more then 0 bytes.
        networkActive = (c > 0);
    }
}

void handleCommand(void *conn, const char *buffer) {
    // The first byte contains the command
    char cmd = buffer[1];
    uint32_t cmdParam[2];  // params not needed

    // Copy over the parameters.
    // memcpy(cmdParam, &buffer[2], sizeof(cmdParam));

    TPacket commandPacket;

    commandPacket.packetType = PACKET_TYPE_COMMAND;
    commandPacket.params[0] = 1;
    commandPacket.params[1] = 100;

    // printf("COMMAND RECEIVED: %c %d %d\n", cmd, cmdParam[0], cmdParam[1]);

    switch (cmd) {
        case 'w':
        case 'W':
            commandPacket.command = COMMAND_FORWARD;
            uartSendPacket(&commandPacket);
            break;
        case 'a':
        case 'A':
        case 's':
        case 'S':
        case 'd':
        case 'D':
        case 'f':
        case 'F':
        case 't':
        case 'T':
            commandPacket.data[0] = cmd;
            uartSendPacket(&commandPacket);
            break;

            // case 'g':
            // case 'G':
            //     commandPacket.command = COMMAND_GET_STATS;
            //     uartSendPacket(&commandPacket);
            //     break;

        default:
            printf("Bad command\n");
    }
}

void handleNetworkData(void *conn, const char *buffer, int len) {
    /* Note: A problem with our design is that we actually get data to be
       written to the SSL network from the serial port. I.e. we send a command
       to the Arduino, get back a status, then write to the TLS connection.  So
       we do a hack: we assume that whatever we get back from the Arduino is
       meant for the most recent client, so we just simply store conn, which
       contains the TLS connection, in a global variable called tls_conn */

    tls_conn = conn;  // This is used by sendNetworkData

    if (buffer[0] == NET_COMMAND_PACKET) handleCommand(conn, buffer);
}

void sendHello() {
    // Send a hello packet
    TPacket helloPacket;

    helloPacket.packetType = PACKET_TYPE_HELLO;
    uartSendPacket(&helloPacket);
}

// tls thread
void *worker(void *conn) {
    int len;
    char buffer[BUF_LEN];

    while (networkActive) {
        len = sslRead(conn, buffer, sizeof(buffer));  // copy 129 bytes into buffer

        networkActive = (len > 0);

        if (len > 0) {
            printf("received command: %c\n", buffer[1]);  // testing
            handleNetworkData(conn, buffer, len);
            // if (buffer[1] == 'c') {
            //     char data[2];
            //     data[0] = NET_MESSAGE_PACKET;
            //     data[1] = 'p';
            //     tls_conn = conn;
            //     sendNetworkData(data, sizeof(data));
            // }
        }

        else if (len < 0)
            perror("ERROR READING NETWORK: ");
    }

    tls_conn = NULL;
    EXIT_THREAD(conn);
}

// arduino thread
void *uartReceiveThread(void *p) {
    char buffer[PACKET_SIZE];
    int len;
    TPacket packet;
    TResult result;
    int counter = 0;

    while (1) {
        len = serialRead(buffer);  // put into buffer
        counter += len;            // no. of bytes received
        if (len > 0) {
            result = deserialize(buffer, len, &packet);  // put into packet

            if (result == PACKET_OK) {
                counter = 0;
                handleUARTPacket(&packet);
            } else if (result != PACKET_INCOMPLETE) {
                printf("PACKET ERROR\n");
                handleError(result);
            }
        }
    }
}

int main() {
    pthread_t serThread;

    printf("\nALEX REMOTE SUBSYSTEM\n\n");

    printf("Opening Serial Port\n");

    // connect to arduino
    startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 1);

    printf("Done. Waiting 3 seconds for Arduino to reboot\n");
    sleep(3);

    printf("DONE. Starting Serial Listener\n");
    pthread_create(&serThread, NULL, uartReceiveThread, NULL);

    printf("Starting Alex Server\n");

    networkActive = 1;

    // connect to laptop
    createServer(KEY_FNAME, CERT_FNAME, SERVER_PORT, &worker, CA_CERT_FNAME, CLIENT_NAME, 1);

    printf("DONE. Sending HELLO to Arduino\n");
    sendHello();
    printf("DONE.\n");

    while (server_is_running())
        ;
}