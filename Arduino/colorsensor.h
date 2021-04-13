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
    DDRD |= 0b10000100;
    DDRB |= 0b00000101;
    DDRB &= ~(1 << 1);

    PORTD |= 1 << 7;
    PORTD &= ~(1 << 2);
}

int getRedPW() {
    PORTB &= ~(1 << 2);  // S2
    PORTB &= ~(1 << 0);  // S3

    int PW;
    PW = pulseIn(sensorOut, LOW);

    return PW;
}

// Function to read Green Pulse Widths
int getGreenPW() {
    PORTB |= (1 << 2);  // S2
    PORTB |= (1 << 0);  // S3

    int PW;
    PW = pulseIn(sensorOut, LOW);

    return PW;
}

int senseColor() {
    redPW = getRedPW();
    redValue = map(redPW, redMin, redMax, 255, 0);
    delay(200);

    greenPW = getGreenPW();
    greenValue = map(greenPW, greenMin, greenMax, 255, 0);
    delay(200);

    if (redValue - greenValue > 500) {
        return 1;
    } else {
        return 0;
    }
}