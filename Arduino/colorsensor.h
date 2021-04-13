#define S1 2
#define S0 7
#define S3 8
#define S2 10
#define sensorOut 9

// Calibration Values
int redMin = 80;     // Red minimum value
int redMax = 95;     // Red maximum value
int greenMin = 120;  // Green minimum value
int greenMax = 130;  // Green maximum value
int blueMin = 115;   // Blue minimum value
int blueMax = 125;   // Blue maximum value

// Variables for Color Pulse Width Measurements
int redPW = 0;
int greenPW = 0;
int bluePW = 0;

// Variables for final Color values
int redValue;
int greenValue;
int blueValue;

void setupColorSensor() {
    // pinMode(S0, OUTPUT);
    // pinMode(S1, OUTPUT);
    // pinMode(S2, OUTPUT);
    // pinMode(S3, OUTPUT);
    // pinMode(sensorOut, INPUT);

    // digitalWrite(S0, HIGH);
    // digitalWrite(S1, LOW);
    DDRD |= 0b10000100;
    DDRB |= 0b00000101;
    DDRB &= ~(1 << 1);

    PORTD |= 1 << 7;
    PORTD &= ~(1 << 2);
}

int getRedPW() {
    // Set sensor to read Red only
    // digitalWrite(S2, LOW);
    // digitalWrite(S3, LOW);

    PORTB &= ~(1 << 2);  // S2
    PORTB &= ~(1 << 0);  // S3
    // Define integer to represent Pulse Width
    int PW;
    // Read the output Pulse Width
    PW = pulseIn(sensorOut, LOW);
    // Return the value
    return PW;
}

// Function to read Green Pulse Widths
int getGreenPW() {
    // Set sensor to read Green only
    // digitalWrite(S2, HIGH);
    // digitalWrite(S3, HIGH);

    PORTB |= (1 << 2);  // S2
    PORTB |= (1 << 0);  // S3
    // Define integer to represent Pulse Width
    int PW;
    // Read the output Pulse Width
    PW = pulseIn(sensorOut, LOW);
    // Return the value
    return PW;
}

// Function to read Blue Pulse Widths
// int getBluePW() {
//     // Set sensor to read Blue only
//     digitalWrite(S2, LOW);
//     digitalWrite(S3, HIGH);
//     // Define integer to represent Pulse Width
//     int PW;
//     // Read the output Pulse Width
//     PW = pulseIn(sensorOut, LOW);
//     // Return the value
//     return PW;
// }

int senseColor() {
    redPW = getRedPW();
    // Map to value from 0-255
    redValue = map(redPW, redMin, redMax, 255, 0);
    // Delay to stabilize sensor
    delay(200);

    // Read Green value
    greenPW = getGreenPW();
    // Map to value from 0-255
    greenValue = map(greenPW, greenMin, greenMax, 255, 0);
    // Delay to stabilize sensor
    delay(200);

    //   // Read Blue value
    //   bluePW = getBluePW();
    //   // Map to value from 0-255
    //   blueValue = map(bluePW, blueMin,blueMax,255,0);

    int diff = redValue - greenValue;
    if (diff > 500) {
        return 1;
    } else {
        return 0;
    }
}