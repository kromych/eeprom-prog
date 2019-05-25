// To be able to write to the serial port:
//  sudo usermod -a -G dialout $USER
// and re-login

// Defines

#define SHIFT_DATA  2
#define SHIFT_CLK   3
#define SHIFT_LATCH 4

// Set up the board

void setup()
{
    // Set up in modes

    pinMode(SHIFT_DATA, OUTPUT);
    pinMode(SHIFT_CLK, OUTPUT);
    pinMode(SHIFT_LATCH, OUTPUT);

    // Output data to the shift registers

    shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, 0xff);

    digitalWrite(SHIFT_LATCH, LOW);
    digitalWrite(SHIFT_LATCH, HIGH);
    digitalWrite(SHIFT_LATCH, LOW);
}

// Run repeatedly

void loop()
{

}
