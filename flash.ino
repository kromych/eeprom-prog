// To be able to write to the serial port:
//  sudo usermod -a -G dialout $USER
// and re-login

// Defines

#define SHIFT_DATA  2
#define SHIFT_CLK   3
#define SHIFT_LATCH 4

#define EEPROM_D0   5
#define EEPROM_D1   6
#define EEPROM_D2   7
#define EEPROM_D3   8
#define EEPROM_D4   9
#define EEPROM_D5   10
#define EEPROM_D6   11
#define EEPROM_D7   12

/**
 * Set teh adddress
 */
void setAddress(int address, bool outputEnable)
{
    // Output data to the shift registers

    shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, 0x00);
    shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, 0x00);

    shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, ((address >> 8) & 0xff) | (outputEnable ? 0x00 : 0x80));
    shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, (address & 0xff));

    digitalWrite(SHIFT_LATCH, LOW);
    digitalWrite(SHIFT_LATCH, HIGH);
    digitalWrite(SHIFT_LATCH, LOW);
}

/**
 * Read EEPROM
 */
byte readEEPROM(int address)
{
    byte data = 0;

    setAddress(address, /*outputEnable*/ true);

    for (int pin = EEPROM_D7; pin >= EEPROM_D0; --pin)
    {
        data = (data << 1) | digitalRead(pin);
    }

    return data;
} 

/**
 * Switch data I/O mode
 */
void setDataMode(int mode)
{
    for (int pin = EEPROM_D7; pin >= EEPROM_D0; --pin)
    {
        pinMode(pin, mode);
    }
}

/** 
 * Set up the board
 */
void setup()
{
    // Set up pin modes

    pinMode(SHIFT_DATA, OUTPUT);
    pinMode(SHIFT_CLK, OUTPUT);
    pinMode(SHIFT_LATCH, OUTPUT);

    setDataMode(INPUT);

    Serial.begin(57600); // Had to set that in the Baud Rate dialog, too
    Serial.println(readEEPROM(0));
}

/** 
 * Run repeatedly
 */
void loop()
{

}
