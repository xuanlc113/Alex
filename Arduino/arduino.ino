#include <serialize.h>

#include "packet.h"
#include "constants.h"
#include "movement.h"
#include "interrupts.h"

// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV 1

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC 1


/*
      Alex's State Variables
*/

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftTicks;
volatile unsigned long rightTicks;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

/*

   Alex Communication Routines.

*/

TResult readPacket(TPacket *packet)
{
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);
}


//probably don't need this
//void sendStatus(){}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful for debugging.
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

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad magic number.
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad checksum.
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its command sent to us.
  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
   Setup and start codes for serial communications

*/
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

//void startSerial(){}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count = 0;

  while (Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

/*
   Alex's motor drivers.

*/

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:
       A1IN - Pin 5, PD5, OC0B
       A2IN - Pin 6, PD6, OC0A
       B1IN - Pin 10, PB2, OC1B
       B2In - pIN 11, PB3, OC2A
  */
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
}



/*
   Alex's setup and run codes

*/

// Clears all our counters
void clearCounters()
{
  leftTicks = 0;
  rightTicks = 0;
  leftRevs = 0;
  rightRevs = 0;
  forwardDist = 0;
  reverseDist = 0;
}

// Clears one particular counter
void clearOneCounter(int which)
{
  switch (which)
  {
    case 0:
      clearCounters();
      break;

    case 1:
      leftTicks = 0;
      break;

    case 2:
      rightTicks = 0;
      break;

    case 3:
      leftRevs = 0;
      break;

    case 4:
      rightRevs = 0;
      break;

    case 5:
      forwardDist = 0;
      break;

    case 6:
      reverseDist = 0;
      break;
  }
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch (command->data[0])//data[0] can be w, a, s, d, f, t
  {
    case 'w':
      sendOK();
      forward((float)1, (float)1); //distance = 1, speed = 1
      Serial.print("w");
      break;

    case 's':
      sendOK();
      reverse((float)10, (float)10);
      Serial.print("s");
      break;

    case 'a':
      sendOK();
      left((float)10, (float)10);
      Serial.print("a");
      break;

    case 'd':
      sendOK();
      right((float)10, (float)10);
      Serial.print("d");
      break;

    case 'f':
      sendOK();
      Serial.print("f");
      break;

    case 't':
      sendOK();
      stop();
      Serial.print("t");
      break;

    //    case COMMAND_GET_STATS:
    //      sendStatus();
    //      break;
    //
    //    case COMMAND_CLEAR_STATS:
    //      sendOK();
    //      clearOneCounter(command->params[0]);
    //      break;

    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit = 0;

  while (!exit)
  {
    TPacket hello;
    TResult result;

    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {

        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}

void setup()
{
  // put your setup code here, to run once:

  cli();
  //setupEINT();
  setupSerial();
  //startSerial();
  setupMotors();
  startMotors();
  //enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
  {
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

void loop()
{

  // Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

  // forward(0, 100);

  // Uncomment the code below for Week 9 Studio 2

  // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK)
    handlePacket(&recvPacket);
  else if (result == PACKET_BAD)
  {
    sendBadPacket();
  }
  else if (result == PACKET_CHECKSUM_BAD)
  {
    sendBadChecksum();
  }
}
