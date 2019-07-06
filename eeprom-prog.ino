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
 * Program the decoder for a 7-segment display
 */
void program7SegmentDispay()
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
 * Layout of the microcode
 * -----------------------
 *
 * Address has 11 (eleven) bits:
 *
 *  10 | 09 | 08 | 07 | 06 | 05 | 04 | 03 | 02 | 01 | 00
 * ----+----+----+----+----+----+----+----+----+----+---
 *  00 | CF | ZF | CS | I3 | I2 | I1 | I0 | T2 | T1 | T0
 *
 * where:
 *    CF, ZF - Carry Flag, Zero Flag
 *    CS     - Chip Select. Two chips are used to provide 16-bit wide output
 *             for control words
 *    I3..I0 - Instruction opcode
 *    T2..T0 - Micro-instruction number
 *
 * Control words have 16 bits:
 *
 *  15 | 14 | 13 | 12 | 11 | 10 | 09 | 08 | 07 | 06 | 05 | 04 | 03 | 02 | 01 | 00
 * ----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----
 *  EO | SU | BI | OI | CE | CO | J  | FI | HLT| MI | RI | RO | IO | II | AI | AO
 *
 *  There can be 16 instructions, each requires 5 micro-instructions, and that
 *  maps to 2-byte control word. 16*5*2 byte -> 160 bytes
 *
 */

#define EO      (1 << 15) // Sum out to the bus
#define SU      (1 << 14) // Subtract
#define BI      (1 << 13) // Register B in from the bus
#define OI      (1 << 12) // Output register in from the bus
#define CE      (1 << 11) // Program counter enable
#define CO      (1 << 10) // Program counter out to the bus
#define J       (1 << 9)  // Jump == Program counter in from the bus
#define FI      (1 << 8)  // Flag register in from the bus
#define HLT     (1 << 7)  // Halt
#define MI      (1 << 6)  // Memory register in from the bus
#define RI      (1 << 5)  // RAM content in from the bus
#define RO      (1 << 4)  // RAM content out to the bus
#define IO      (1 << 3)  // Instruction regsiter out to the bus
#define II      (1 << 2)  // Instruction regsiter in from the bus
#define AI      (1 << 1)  // Regsiter A in from the bus
#define AO      (1 << 0)  // Regsiter A out to the bus
#define NOP     0         // No operation

#define OPCODE_COUNT       16
#define MICROINSTR_COUNT   5

/**
 * Micro-code for the Instruction Set Architecture as
 * a seqeunce of control words.
 * Each instruction contains instruction opcode in the 4 MSB bits and
 * may contain memory address in the 4 LSB bits. Thus 16 bytes of memory
 * can be addressed.
 */
int MicroCode[OPCODE_COUNT][MICROINSTR_COUNT] =
{
    /* 4b'0000 NOP */ { /*1*/ CO|MI, /*2*/ RO|II|CE, /*3*/ NOP,   /*4*/ NOP,   /*5*/ NOP      },
    /* 4b'0001 LDA */ { /*1*/ CO|MI, /*2*/ RO|II|CE, /*3*/ IO|MI, /*4*/ RO|AI, /*5*/ NOP      },
    /* 4b'0010 ADD */ { /*1*/ CO|MI, /*2*/ RO|II|CE, /*3*/ IO|MI, /*4*/ RO|BI, /*5*/ EO|AI    },
    /* 4b'0011 SUB */ { /*1*/ CO|MI, /*2*/ RO|II|CE, /*3*/ IO|MI, /*4*/ RO|BI, /*5*/ EO|AI|SU },
    /* 4b'0100 STA */ { /*1*/ CO|MI, /*2*/ RO|II|CE, /*3*/ IO|MI, /*4*/ AO|RI, /*5*/ NOP      },
    /* 4b'0101 LDI */ { /*1*/ CO|MI, /*2*/ RO|II|CE, /*3*/ IO|AI, /*4*/ NOP,   /*5*/ NOP      },
    /* 4b'0110 JMP */ { /*1*/ CO|MI, /*2*/ RO|II|CE, /*3*/ IO|J,  /*4*/ NOP,   /*5*/ NOP      },
    /* 4b'0111 HLT */ { /*1*/ HLT,   /*2*/ HLT,      /*3*/ HLT,   /*4*/ HLT,   /*5*/ HLT      },
    /* 4b'1000 HLT */ { /*1*/ HLT,   /*2*/ HLT,      /*3*/ HLT,   /*4*/ HLT,   /*5*/ HLT      },
    /* 4b'1001 HLT */ { /*1*/ HLT,   /*2*/ HLT,      /*3*/ HLT,   /*4*/ HLT,   /*5*/ HLT      },
    /* 4b'1010 HLT */ { /*1*/ HLT,   /*2*/ HLT,      /*3*/ HLT,   /*4*/ HLT,   /*5*/ HLT      },
    /* 4b'1011 HLT */ { /*1*/ HLT,   /*2*/ HLT,      /*3*/ HLT,   /*4*/ HLT,   /*5*/ HLT      },
    /* 4b'1100 HLT */ { /*1*/ HLT,   /*2*/ HLT,      /*3*/ HLT,   /*4*/ HLT,   /*5*/ HLT      },
    /* 4b'1101 HLT */ { /*1*/ HLT,   /*2*/ HLT,      /*3*/ HLT,   /*4*/ HLT,   /*5*/ HLT      },
    /* 4b'1110 OUT */ { /*1*/ CO|MI, /*2*/ RO|II|CE, /*3*/ AO|OI, /*4*/ NOP,   /*5*/ NOP      },
    /* 4b'1111 HLT */ { /*1*/ HLT,   /*2*/ HLT,      /*3*/ HLT,   /*4*/ HLT,   /*5*/ HLT      },
};

/**
 * Example program 1, starts at address 0: 
 * 
 * 0: LDA  14      ; 8b'0001_1110 Load byte from address 14 into register A
 * 1: ADD  15      ; 8b'0010_1111 Add byte from address 15 to register A
 * 2: OUT          ; 8b'1110_0000 Output to the 7-segment display
 * 3: HLT          ; 8b'1111_0000 Halt
 * 
 */

/**
 * Example program 2, starts at address 0: 
 * 
 * 0: LDI  3       ; 8b'0101_0011 Load 3 into register A
 * 1: STA  15      ; 8b'0100_1111 Store byte from register A to address 15
 * 2: LDI  0       ; 8b'0101_0000 Load 0 into register A
 * 3: ADD  15      ; 8b'0010_1111 Add byte from address 15 to register A
 * 4: OUT          ; 8b'1110_0000 Output to the 7-segment display
 * 5: JMP  3       ; 8b'0110_0011 Jump to the instruction at the address 3 (ADD 15)
 * 
 */

void programMicrocode()
{

    for (unsigned opcode = 0; opcode < OPCODE_COUNT; ++opcode)
    {
        for (unsigned microInstrIdx = 0; microInstrIdx < MICROINSTR_COUNT; ++microInstrIdx)
        {
            // Compute address of the least significant byte of the control word 
            // in the microcode

            int addressLow = microInstrIdx | (opcode << 3);

            int controlWord = MicroCode[opcode][microInstrIdx];

            // LSB of the control word

            int controlWordLsb = controlWord & 0xff;

            // Write LSB

            writeEEPROM(addressLow, controlWordLsb);

            // Compute address of the most significant byte of the control word 
            // in the microcode

            int addressHigh = 128 + addressLow;

            // MSB of the control word

            int controlWordMsb = (controlWord >> 8) & 0xff;

            // Write MSB

            writeEEPROM(addressHigh, controlWordMsb);
        }
    }
};

/**
 * Program EEPROM
 */
void programEEPROM()
{
    programMicrocode();
}

/**
 * Set up the board
 */
void setup()
{
    // Had to set that in the Baud Rate dialog, too

    Serial.begin(57600);

    // Set up pin modes

    Serial.println("Initializing pins");

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
