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

#define WRITE_EN    13  // Active low

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
 * Set the adddress
 */
void setAddress(int address, bool outputEnable)
{
    // Output data to the shift registers

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

    setDataMode(INPUT);
    setAddress(address, /*outputEnable*/true);

    for (int pin = EEPROM_D7; pin >= EEPROM_D0; --pin)
    {
        data = (data << 1) | digitalRead(pin);
    }

    setAddress(address, /*outputEnable*/false);

    return data;
} 

/**
 * Write EEPROM
 */
void writeEEPROM(int address, byte data)
{
    // Prevent the EEPROM and Arduino from trying to drive the 
    // bus at the same time

    delay(10);

    setAddress(address, /*outputEnable*/false);
    setDataMode(OUTPUT);

    for (int pin = EEPROM_D0; pin <= EEPROM_D7; ++pin)
    {
        digitalWrite(pin, data & 1);
        data >>= 1;
    }

    // WE is active-low so pulsing it means going
    // HIGH->LOW->HIGH

    digitalWrite(WRITE_EN, LOW);
    delayMicroseconds(1);
    digitalWrite(WRITE_EN, HIGH);
    delay(10);

    setDataMode(INPUT);
}

/**
 * Set the EEPROM content
 */
void setEEPROM(int value)
{
    for (int address = 0; address <= 2047; ++address)
    {
        writeEEPROM(address, value);
    }
} 

/**
 * Dump content
 */
void dumpEEPROM()
{
    for (int base = 0; base <= 255; base += 16)
    {
        byte data[16];

        for (int offset = 0; offset <= 15; ++offset)
        {
            data[offset] = readEEPROM(base + offset);
        }

        char buf[80];

        sprintf(
            buf, 
            "%03x:  %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x",
            base,
            data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], 
            data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]
        );

        Serial.println(buf);
    }
}

/**
 * Program EEPROM
 */ 
void programEEPROM()
{
    // Bit patterns for the digits 0..9

    byte digits[] = { 0x7e, 0x30, 0x6d, 0x79, 0x33, 0x5b, 0x5f, 0x70, 0x7f, 0x7b };

    Serial.println("Programming ones place");

    for (int value = 0; value <= 255; value += 1) 
    {
        writeEEPROM(value, digits[value % 10]);
    }

    Serial.println("Programming tens place");

    for (int value = 0; value <= 255; value += 1) 
    {
        writeEEPROM(value + 256, digits[(value / 10) % 10]);
    }

    Serial.println("Programming hundreds place");

    for (int value = 0; value <= 255; value += 1) 
    {
        writeEEPROM(value + 512, digits[(value / 100) % 10]);
    }

    Serial.println("Programming sign");

    for (int value = 0; value <= 255; value += 1) 
    {
        writeEEPROM(value + 768, 0);
    }

    Serial.println("Programming ones place (twos complement)");

    for (int value = -128; value <= 127; value += 1) 
    {
        writeEEPROM((byte)value + 1024, digits[abs(value) % 10]);
    }

    Serial.println("Programming tens place (twos complement)");

    for (int value = -128; value <= 127; value += 1) 
    {
        writeEEPROM((byte)value + 1280, digits[abs(value / 10) % 10]);
    }

    Serial.println("Programming hundreds place (twos complement)");

    for (int value = -128; value <= 127; value += 1) 
    {
        writeEEPROM((byte)value + 1536, digits[abs(value / 100) % 10]);
    }

    Serial.println("Programming sign (twos complement)");

    for (int value = -128; value <= 127; value += 1) 
    {
        if (value < 0) 
        {
            writeEEPROM((byte)value + 1792, 0x01);
        } 
        else 
        {
            writeEEPROM((byte)value + 1792, 0);
        }
    }
}

/** 
 * Set up the board
 */
void setup()
{
    // Had to set that in the Baud Rate dialog, too

    Serial.begin(57600);

    // Set up pin modes

    Serial.println("Initializng pins");

    pinMode(SHIFT_DATA, OUTPUT);
    pinMode(SHIFT_CLK, OUTPUT);
    pinMode(SHIFT_LATCH, OUTPUT);

    digitalWrite(WRITE_EN, HIGH);   // Sets the pull-up resistor
    pinMode(WRITE_EN, OUTPUT);      // Set the mode

    setDataMode(INPUT);
    setAddress(0x00, false);

    Serial.println("Erasing EEPROM");

    setEEPROM(0xff);

    Serial.println("Programming EEPROM");
   
    programEEPROM();

    Serial.println("EEPROM content:");

    dumpEEPROM();
}

/** 
 * Run repeatedly
 */
void loop()
{

}
