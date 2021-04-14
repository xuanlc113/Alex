#include <ncurses.h>

#include "constants.h"
#include "make_tls_client.h"
#include "netconstants.h"

#define CA_CERT_FNAME "cert/signing.pem"
#define CLIENT_CERT_FNAME "controlkey/laptop.crt"
#define CLIENT_KEY_FNAME "controlkey/laptop.key"
#define SERVER_NAME_ON_CERT "www.alex.com"

// Tells us that the network is running.
static volatile int networkActive = 0;

void handleError(const char *buffer) {
    switch (buffer[1]) {
        case RESP_OK:
            // printf("Command / Status OK\n");
            break;

        case RESP_BAD_PACKET:
            printf("BAD MAGIC NUMBER FROM ARDUINO\n");
            break;

        case RESP_BAD_CHECKSUM:
            printf("BAD CHECKSUM FROM ARDUINO\n");
            break;

        case RESP_BAD_COMMAND:
            printf("PI SENT BAD COMMAND TO ARDUINO\n");
            break;

        case RESP_BAD_RESPONSE:
            printf("PI GOT BAD RESPONSE FROM ARDUINO\n");
            break;

        default:
            printf("PI IS CONFUSED!\n");
    }
}

void handleStatus(const char *buffer) {
    int32_t data[16];
    memcpy(data, &buffer[1], sizeof(data));

    printf("\r\ncolor: %s\r\n", data[0] ? "red" : "green");
}

void handleMessage(const char *buffer) { printf("\r\nMESSAGE FROM ALEX: %s\r\n", &buffer[1]); }

void handleCommand(const char *buffer) {
    // We don't do anything because we issue commands
    // but we don't get them. Put this here
    // for future expansion
}

void handleNetwork(const char *buffer, int len) {
    // The first byte is the packet type
    int type = buffer[0];

    switch (type) {
        case NET_ERROR_PACKET:
            handleError(buffer);
            break;

        case NET_STATUS_PACKET:
            handleStatus(buffer);
            break;

        case NET_MESSAGE_PACKET:
            handleMessage(buffer);
            break;

        case NET_COMMAND_PACKET:
            handleCommand(buffer);
            break;
    }
}

void sendData(void *conn, const char *buffer, int len) {
    int c;
    if (networkActive) {
        c = sslWrite(conn, buffer, len);
        networkActive = (c > 0);
    }
}

void *readerThread(void *conn) {
    char buffer[128];
    int len;

    while (networkActive) {
        len = sslRead(conn, buffer, sizeof(buffer));

        networkActive = (len > 0);

        if (networkActive) handleNetwork(buffer, len);
    }

    printf("Exiting network listener thread\n");
    stopClient();
    EXIT_THREAD(conn);
}

void *writerThread(void *conn) {
    int quit = 0;

    initscr();
    printf("Command (WASD, f=scan q=exit)\n");

    while (!quit) {
        char ch;
        ch = getch();

        char buffer[2];

        buffer[0] = NET_COMMAND_PACKET;
        switch (ch) {
            case 'w':
            case 'W':
            case 'a':
            case 'A':
            case 's':
            case 'S':
            case 'd':
            case 'D':
            case 'x':
            case 'X':
            case 'c':
            case 'C':
                buffer[1] = ch;
                sendData(conn, buffer, sizeof(buffer));
                break;
            case 'f':
            case 'F':
                buffer[1] = ch;
                sendData(conn, buffer, sizeof(buffer));
                break;
            case 'q':
            case 'Q':
                quit = 1;
                break;
            default:
                printf("BAD COMMAND\n");
        }
        flushinp();
        usleep(500000);
    }

    printf("Exiting keyboard thread\n");

    endwin();
    stopClient();
    EXIT_THREAD(conn);
}

void connectToServer(const char *serverName, int portNum) {
    createClient(serverName, portNum, 1, CA_CERT_FNAME, SERVER_NAME_ON_CERT, 1, CLIENT_CERT_FNAME,
                 CLIENT_KEY_FNAME, readerThread, writerThread);
}

int main(int ac, char **av) {
    if (ac != 3) {
        fprintf(stderr, "\n\n%s <IP address> <Port Number>\n\n", av[0]);
        exit(-1);
    }

    networkActive = 1;
    connectToServer(av[1], atoi(av[2]));

    while (client_is_running())
        ;
    printf("\nMAIN exiting\n\n");
}
