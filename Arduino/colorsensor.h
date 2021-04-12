// #include <Adafruit_TCS34725.h>

#include <Wire.h>

/* Example code for the Adafruit TCS34725 breakout library */

/* Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground */

/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

// tcs3200 tcs(8, 9, 10, 12, 2);
/* Initialise with specific int time and gain values */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

#define S0 8
#define S1 9
#define S2 10
#define S3 12
#define output 2

int colorRead(char, int);

void setupColorSensor() {
    // if (tcs.begin()) {
    //     return;
    // } else {
    //     return;
    // }
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(output, INPUT);
}

// void disableColorSensor() { tcs.disable(); }

int senseColor() {
    int red = colorRead('r', 20);  // reads color value for red

    int green = colorRead('g', 20);  // reads color value for green

    int blue = colorRead('b', 20);
    if (red - green > 0) {
        return 1;
    } else {
        return 0;
    }
}

int colorRead(char color, int scaling) {
    switch (scaling) {
        case 0:
            digitalWrite(S0, LOW);  // Set scaling to 0%(scaling is turned OFF)
            digitalWrite(S1, LOW);
            break;

        case 2:
            digitalWrite(S0, LOW);  // Set scaling to 2%
            digitalWrite(S1, HIGH);
            break;

        case 20:  // Set scaling to 20%
            digitalWrite(S0, HIGH);
            digitalWrite(S1, LOW);
            break;

        case 100:  // Set scaling to 100%
            digitalWrite(S0, HIGH);
            digitalWrite(S1, HIGH);
            break;

        default:  // Set default scaling (default scaling is 20%)
            digitalWrite(S0, LOW);
            digitalWrite(S1, LOW);
            break;
    }

    switch (color) {
        case 'r':  // Setting red filtered photodiodes to be read
            digitalWrite(S2, LOW);
            digitalWrite(S3, LOW);
            break;

        case 'b':  // Setting blue filtered photodiodes to be read
            digitalWrite(S2, LOW);
            digitalWrite(S3, HIGH);
            break;

        case 'c':  // Setting clear photodiodes(no filters on diodes) to be read
            digitalWrite(S2, HIGH);
            digitalWrite(S3, LOW);
            break;

        case 'g':  // Setting green filtered photodiodes to be read
            digitalWrite(S2, HIGH);
            digitalWrite(S3, HIGH);
            break;

        default:
            digitalWrite(S2, HIGH);
            digitalWrite(S3, LOW);
            break;
    }

    unsigned long duration;

    duration = pulseIn(output, LOW);

    if (duration != 0) {
        return 1000 / duration;  // Reads and returns the frequency of selected color
    } else {
        return 0;
    }
}

// int senseColor() {
//     float r, g, b;
//     tcs.getRGB(&r, &g, &b);
//     int redval = r - g;

//     if (redval > 0) {
//         return 1;
//     } else {
//         return 0;
//     }
// }

// void loop(void) {
//     float r, g, b, c, colorTemp, lux;

//     // tcs.getRawData(&r, &g, &b, &c);
//     // // colorTemp = tcs.calculateColorTemperature(r, g, b);
//     // colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
//     // lux = tcs.calculateLux(r, g, b);

//     tcs.getRGB(&r, &g, &b);

//     Serial.print("R: ");
//     Serial.print(r, DEC);
//     Serial.print(" ");
//     Serial.print("G: ");
//     Serial.print(g, DEC);
//     Serial.print(" ");
//     Serial.print("B: ");
//     Serial.print(b, DEC);
//     Serial.print(" ");
//     delay(500);
// }

// #include <Wire.h>

// #include "Adafruit_TCS34725.h"

// // Pick analog outputs, for the UNO these three work well
// // use ~560  ohm resistor between Red & Blue, ~1K for green (its brighter)
// #define redpin 3
// #define greenpin 5
// #define bluepin 6
// // for a common anode LED, connect the common pin to +5V
// // for common cathode, connect the common to ground

// // set to false if using a common cathode LED
// #define commonAnode true

// // our RGB -> eye-recognized gamma color
// byte gammatable[256];

// Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// // void setup() {
// //}

// // The commented out code in loop is example of getRawData with clear value.
// // Processing example colorview.pde can work with this kind of data too, but It requires manual
// // conversion to [0-255] RGB value. You can still uncomments parts of colorview.pde and play with
// // clear value.
// // void loop() {
// //}

// void setupColorSensor() {
//     // Serial.begin(9600);
//     // Serial.println("Color View Test!");

//     if (tcs.begin()) {
//         // Serial.println("Found sensor");
//     } else {
//         Serial.println("No TCS34725 found ... check your connections");
//         while (1)
//             ;  // halt!
//     }

//     // use these three pins to drive an LED
// #if defined(ARDUINO_ARCH_ESP32)
//     ledcAttachPin(redpin, 1);
//     ledcSetup(1, 12000, 8);
//     ledcAttachPin(greenpin, 2);
//     ledcSetup(2, 12000, 8);
//     ledcAttachPin(bluepin, 3);
//     ledcSetup(3, 12000, 8);
// #else
//     pinMode(redpin, OUTPUT);
//     pinMode(greenpin, OUTPUT);
//     pinMode(bluepin, OUTPUT);
// #endif

//     // thanks PhilB for this gamma table!
//     // it helps convert RGB colors to what humans see
//     for (int i = 0; i < 256; i++) {
//         float x = i;
//         x /= 255;
//         x = pow(x, 2.5);
//         x *= 255;

//         if (commonAnode) {
//             gammatable[i] = 255 - x;
//         } else {
//             gammatable[i] = x;
//         }
//         // Serial.println(gammatable[i]);
//     }
// }

// int senseColor() {
//     float red, green, blue;

//     tcs.setInterrupt(false);  // turn on LED

//     delay(60);  // takes 50ms to read

//     tcs.getRGB(&red, &green, &blue);

//     tcs.setInterrupt(true);  // turn off LED

//     int redval = red - green;

//     if (redval > 0) {
//         return 1;
//     } else {
//         return 0;
//     }

//     // Serial.print("R:\t"); Serial.print(redval);
//     //  Serial.print("\tG:\t"); Serial.print(int(green));
//     //  Serial.print("\tB:\t"); Serial.print(int(blue));

//     // Serial.print("\n");

// #if defined(ARDUINO_ARCH_ESP32)
//     ledcWrite(1, gammatable[(int)red]);
//     ledcWrite(2, gammatable[(int)green]);
//     ledcWrite(3, gammatable[(int)blue]);
// #else
//     analogWrite(redpin, gammatable[(int)red]);
//     analogWrite(greenpin, gammatable[(int)green]);
//     analogWrite(bluepin, gammatable[(int)blue]);
// #endif
// }
