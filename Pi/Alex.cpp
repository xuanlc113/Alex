#include "server_lib/constants.h"
#include "server_lib/make_tls_server.h"
#include "server_lib/netconstants.h"
#include "server_lib/packet.h"
#include "server_lib/serial.h"
#include "server_lib/serialize.h"
#include "server_lib/tls_common_lib.h"

#define PORT_NAME "/dev/ttyACM0"

#define BAUD_RATE B57600

// TLS Port Number
#define SERVER_PORT 5000

// CA Keys
#define KEY_FNAME "pikey/alex.key"
#define CERT_FNAME "pikey/alex.crt"
#define CA_CERT_FNAME "cert/signing.pem"
#define CLIENT_NAME "www.control.com"

// Our network buffer consists of 1 byte of packet type, and 128 bytes of data
#define BUF_LEN 129

static volatile int networkActive;

static void *tls_conn = NULL;

/*

Alex Serial Routines to the Arduino

*/

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
        case PACKET_TYPE_COMMAND:
            // Only we send command packets, so ignore
            break;

        case PACKET_TYPE_RESPONSE:
            handleResponse(packet);
            break;

        case PACKET_TYPE_ERROR:
            handleErrorResponse(packet);
            break;

        case PACKET_TYPE_MESSAGE:
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

void *uartReceiveThread(void *p) {
    char buffer[PACKET_SIZE];
    int len;
    TPacket packet;
    TResult result;
    int counter = 0;

    while (1) {
        len = serialRead(buffer);
        counter += len;
        if (len > 0) {
            result = deserialize(buffer, len, &packet);

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

/*

Alex Network Routines

*/

void sendNetworkData(const char *data, int len) {
    if (networkActive) {
        int c;

        printf("WRITING TO CLIENT\n");

        if (tls_conn != NULL) {
            sslWrite(tls_conn, data, len);
        }

        // Network is still active if we can write more then 0 bytes.
        networkActive = (c > 0);
    }
}

void handleCommand(void *conn, const char *buffer) {
    // The first byte contains the command
    char cmd = buffer[1];

    TPacket commandPacket;

    commandPacket.packetType = PACKET_TYPE_COMMAND;
    printf("COMMAND RECEIVED: %c\n", cmd);

    switch (cmd) {
        case 'w':
        case 'W':
            commandPacket.command = COMMAND_FORWARD;
            uartSendPacket(&commandPacket);
            break;

        case 's':
        case 'S':
            commandPacket.command = COMMAND_REVERSE;
            uartSendPacket(&commandPacket);
            break;

        case 'a':
        case 'A':
            commandPacket.command = COMMAND_TURN_LEFT;
            uartSendPacket(&commandPacket);
            break;

        case 'd':
        case 'D':
            commandPacket.command = COMMAND_TURN_RIGHT;
            uartSendPacket(&commandPacket);
            break;

        case 'x':
        case 'X':
            commandPacket.command = COMMAND_STOP;
            uartSendPacket(&commandPacket);
            break;

        case 'f':
        case 'F':
            commandPacket.command = COMMAND_GET_STATS;
            uartSendPacket(&commandPacket);
            break;

        default:
            printf("Bad command\n");
    }
}

void handleNetworkData(void *conn, const char *buffer, int len) {
    /* Note: A problem with our design is that we actually get data to be written
        to the SSL network from the serial port. I.e. we send a command to the Arduino,
        get back a status, then write to the TLS connection.  So we do a hack:
        we assume that whatever we get back from the Arduino is meant for the most
        recent client, so we just simply store conn, which contains the TLS
        connection, in a global variable called tls_conn */

    tls_conn = conn;

    if (buffer[0] == NET_COMMAND_PACKET) handleCommand(conn, buffer);
}

void *worker(void *conn) {
    int len;

    char buffer[BUF_LEN];

    while (networkActive) {
        len = sslRead(conn, buffer, sizeof(buffer));
        // As long as we are getting data, network is active
        networkActive = (len > 0);

        if (len > 0)
            handleNetworkData(conn, buffer, len);
        else if (len < 0)
            perror("ERROR READING NETWORK: ");
    }

    // Reset tls_conn to NULL.
    tls_conn = NULL;
    EXIT_THREAD(conn);
}

void sendHello() {
    // Send a hello packet
    TPacket helloPacket;

    helloPacket.packetType = PACKET_TYPE_HELLO;
    uartSendPacket(&helloPacket);
}

int main() {
    pthread_t serThread;

    printf("\nALEX REMOTE SUBSYSTEM\n\n");

    printf("Opening Serial Port\n");
    startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);
    printf("Done. Waiting 3 seconds for Arduino to reboot\n");
    sleep(3);

    printf("DONE. Starting Serial Listener\n");
    pthread_create(&serThread, NULL, uartReceiveThread, NULL);

    printf("Starting Alex Server\n");

    networkActive = 1;

    createServer(KEY_FNAME, CERT_FNAME, SERVER_PORT, &worker, CA_CERT_FNAME, CLIENT_NAME, 1);

    printf("DONE. Sending HELLO to Arduino\n");
    sendHello();
    printf("DONE.\n");

    while (server_is_running())
        ;
}
