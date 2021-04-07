// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF 6  // Left forward pin
#define LR 5  // Left reverse pin
#define RF 10 // Right forward pin
#define RR 11 // Right reverse pin

// Convert percentages to PWM values
int pwmVal(float speed)
{
    if (speed < 0.0)
        speed = 0;

    if (speed > 100.0)
        speed = 100.0;

    return (int)((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
    int val = pwmVal(speed);

    // For now we will ignore dist and move
    // forward indefinitely. We will fix this
//     in Week 9.

    // LF = Left forward pin, LR = Left reverse pin
    // RF = Right forward pin, RR = Right reverse pin
    // This will be replaced later with bare-metal code.

    analogWrite(LF, val);
    analogWrite(RF, val);
    analogWrite(LR, 0);
    analogWrite(RR, 0);
}


// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{

    int val = pwmVal(speed);

    // For now we will ignore dist and
    // reverse indefinitely. We will fix this
    // in Week 9.

    // LF = Left forward pin, LR = Left reverse pin
    // RF = Right forward pin, RR = Right reverse pin
    // This will be replaced later with bare-metal code.
    analogWrite(LR, val);
    analogWrite(RR, val);
    analogWrite(LF, 0);
    analogWrite(RF, 0);
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
    int val = pwmVal(speed);

    // For now we will ignore ang. We will fix this in Week 9.
    // We will also replace this code with bare-metal later.
    // To turn left we reverse the left wheel and move
    // the right wheel forward.
    analogWrite(LR, val);
    analogWrite(RF, val);
    analogWrite(LF, 0);
    analogWrite(RR, 0);
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
    int val = pwmVal(speed);

    // For now we will ignore ang. We will fix this in Week 9.
    // We will also replace this code with bare-metal later.
    // To turn right we reverse the right wheel and move
    // the left wheel forward.
    analogWrite(RR, val);
    analogWrite(LF, val);
    analogWrite(LR, 0);
    analogWrite(RF, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
    analogWrite(LF, 0);
    analogWrite(LR, 0);
    analogWrite(RF, 0);
    analogWrite(RR, 0);
}
