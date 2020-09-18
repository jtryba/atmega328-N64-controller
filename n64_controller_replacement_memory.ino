/**
 * Gamecube controller to Nintendo 64 adapter
 * by Andrew Brown
 * https://github.com/brownan/Gamecube-N64-Controller/blob/master/gamecube.ino
 * 
 * 
 * Rewritten by jtryba so the atmega328 can be used as a stand alone replacement to the Nintendo CNT-NUS N64 controller chip, with built in rumble, for use in portables.
 * Idealy for the Arduino Nano or Arduino Pro Mini, As long as it uses an ATmega328, ATmega328P, or ATmega168 @ 16Mhz.
 * Optionally supports original N64 encoder joystick or memory pak emulation.
 * (not enough pins for both, we could use a single anolog pin for the d-pad but would require more external resistors, see link below)
 * https://forum.arduino.cc/index.php?topic=8558.0
 * 
 * Note: Holding L and R while pressing start will recalibrate the joystick, this feature was programmed into real n64 controllers, and i liked the idea so i kept it.
 * Note: Holding L and R while pressing Z will switch between rumble mode and memory pack mode.
 * 
 *
 * Emulates a CPack using a 24LC256 32k I2C EEPROM
 * Save manager can be found here:
 * https://bryc.github.io/mempak
 * The above link is where i got an empty cpack.N64 file from (CPack file)
 * after obtaining a fresh cpack.N64 file i converted it to a cpack.MPK using:
 * https://beckabney.com/mk64/mempak.php
 * then wrote the cpack.MPK raw data to the eeprom before use on a n64.
 * 
 * MPK is standard for EverDrives and most virtual mempaks in emulators.
 * DexDrive saves have .N64 extensions.
 * Raphnet-tech_adapter_manager dumps also have .N64
 * 
 * raphnet-tech_adapter_manager can be found here:
 * https://www.raphnet-tech.com/products/adapter_manager/index.php
 * N64 to USB adapter - V3 required to use the above program, that can be found here:
 * https://www.raphnet-tech.com/products/n64_usb_adapter_gen3/index.php
 * 
 * N64 controller protocol can be found here:
 * https://code.google.com/archive/p/micro-64-controller/wikis/Protocol.wiki
 * 
 * 
 * To use, hook up the following to the Atmega (I used the 328P-PU and the 168)
 * Digital I/O 8:  N64 serial line
 * Digital I/O 10: 820 ohm resistor to 2n2222 base, emitter to ground, collector to motor using 1N4007 flyback diode accross motor legs, and finally motor to 5v
 * All appropriate grounding and power lines, joystick, and all buttons (all active low, see pin definitions below)
 * 
 *   /------------\
 *  / O    O     O \
 * | 3.3V Signl GND |
 * |________________|
 *   (Front of N64)
 *
 * The pin-out for the N64 and Gamecube wires can be found here:
 * http://svn.navi.cx/misc/trunk/wasabi/devices/cube64/hardware/cube64-basic.pdf
 * Note: that diagram is not for this project, but for a similar project which
 * uses a PIC microcontroller. However, the diagram does describe the pinouts
 * of the gamecube and N64 wires.
 *
 * Also note: the N64 supplies a 3.3 volt line, but I don't plug that into
 * anything.  The arduino can't run off of that many volts, it needs more, so
 * it's powered externally. Therefore, only two lines
 * from the N64 are used.
 * 
 * Just use the 5v rail to power this in a portable.
 */

//#define USE_ENCODER 1 // uncomment this line to use the orig n64 encoder
#define USE_EEPROM 1 // uncomment this line to use the 24LC256 to emulate a cpack

#define F_CPU 16000000

#define N64_PIN 8
#define RUMBLE_PIN 10
#define N64_HIGH DDRB &= ~0x01
#define N64_LOW DDRB |= 0x01
#define N64_QUERY (PINB & 0x01)

#include <Wire.h>
#include "pins_arduino.h"
#include "crc_table.h"

#ifdef USE_EEPROM
  /*This address is determined by the way your address pins are wired.
  I connected A0 and A1 to Ground and A2 to 5V. To get the address,
  we start with the control code from the datasheet (1010) and add
  the logic state for each address pin in the order A2, A1, A0 (100)
  which gives us 0b1010100, or in Hexadecimal, 0x54*/
  #define EEPROM_ADR 0x54
  
  /*The 24LC256 has a 64-byte page write buffer but N64 writes 32 at a time*/
  #define I2C_PAGE 32
  
  #define I2C_CLOCK 400000
  
  // eeprom page buffers
  byte tempStore[I2C_PAGE];
  byte eepromData[I2C_PAGE];
#endif

// were using a PWM signal to control the rumble motor in order to save battery
#define RUMBLE_FORCE  250 //0-255

#ifdef USE_ENCODER
  #define encoderIx A1
  #define encoderQx A2
  #define encoderIy A4
  #define encoderQy A5
  #define encoderX  // not enough pins
  #define encoderY  // not enough pins
  volatile signed int countx;
  volatile signed int county;
#else
  #define JOY_DEAD      2
  #define JOY_RANGE     400 // 1023 * 0.4 rounded a bit
  #define JOY_X         A1
  #define JOY_Y         A2
  //#define I2C_SCL     A4
  //#define I2C_SDA     A5
#endif

#define BUTTON_COUNT  14
#define BTN_A         0
#define BTN_B         1
#define BTN_C_UP      2
#define BTN_C_DOWN    3
#define BTN_C_LEFT    4
#define BTN_C_RIGHT   5
#define BTN_L         9
#define BTN_R         A0
#define BTN_Z         11
#define PAD_UP        6
#define PAD_DOWN      7
#define PAD_LEFT      12
#define PAD_RIGHT     A3
#define BTN_START     13

// Control sticks:
// N64 expects a signed value from -128 to 128 with 0 being neutral
//
// Additionally, the 64 controllers are relative. Whatever stick position
// it's in when it's powered on is what it reports as 0.
//
// While the joystick data is a signed 8 bit 2s complement we know from Micro
// that controllers only have 160 steps on them and I've had games which screw
// up when given the full 8 bit range.
//
// 160 steps diveded by 2 is 80 steps in each direction however,
// by using the controller_test.rom on official hardware,
// I've found that most controllers report closer to 90-100 anyway
//
// More technical info on joysticks can be found here:
// http://n64devkit.square7.ch/pro-man/pro26/26-02.htm#01
//
#define JOY_MAX_REPORT 100 // 127 for full 8 bit data

int JOY_X_MIN = 0;    // will be calculated later in CalStick()
int JOY_X_MAX = 1023; // will be calculated later in CalStick()
int JOY_Y_MIN = 0;    // will be calculated later in CalStick()
int JOY_Y_MAX = 1023; // will be calculated later in CalStick()

// Zero points for the controller stick
static unsigned char zero_x;
static unsigned char zero_y;

const int btn[BUTTON_COUNT] = 
{
  BTN_A, BTN_B, BTN_Z, BTN_START, PAD_UP, PAD_DOWN, PAD_LEFT, PAD_RIGHT,
  BTN_L, BTN_R, BTN_C_UP, BTN_C_DOWN, BTN_C_LEFT, BTN_C_RIGHT
};

static char n64_raw_dump[281]; // maximum recv is 1+2+32 bytes + 1 bit
// n64_raw_dump does /not/ include the command byte. That gets pushed into
// n64_command:
static unsigned char n64_command;

// bytes to send to the 64
// maximum we'll need to send is 33, 32 for a read request and 1 CRC byte
static unsigned char n64_buffer[33];

static void get_n64_command(void);
static void n64_send(unsigned char *buffer, char length, bool wide_stop);
void ReadInputs(void);
void CalStick(void);
signed int GetStick_x(void);
signed int GetStick_y(void);
#ifdef USE_EEPROM
  static word addrCRC(word address);
  //static byte dataCRC(byte * data);
  void readEEPROMPage(long eeAddress);
  void writeEEPROMPage(long eeAddress);
  long lastSwitch = millis();
#endif
#ifdef USE_ENCODER
  void handleEncoderX(void);
  void handleEncoderX(void);
#endif

static bool rumble = false;
static bool rumble_mode = true;

void setup()
{
  // Communication with the N64 on this pin
  digitalWrite(N64_PIN, LOW);
  pinMode(N64_PIN, INPUT);

  /*
  Serial.begin(9600);
  Serial.println();
  Serial.println("Setup has started!");
  */
  
  // setup I/O
  // stick
  #ifdef USE_ENCODER
    countx=0;
    county=0;
    pinMode(encoderIx, INPUT);
    pinMode(encoderQx, INPUT);
    pinMode(encoderIy, INPUT);
    pinMode(encoderQy, INPUT);
    attachInterrupt(encoderX, handleEncoderX, CHANGE);
    attachInterrupt(encoderY, handleEncoderY, CHANGE); 
  #else
    pinMode(JOY_X, INPUT);
    pinMode(JOY_Y, INPUT);
  #endif
  
  // buttons
  for (int i = 0; i < BUTTON_COUNT; i ++)
  {
    digitalWrite(btn[i], LOW);
    pinMode(btn[i], INPUT_PULLUP);
  }
  
  // rumble
  digitalWrite(RUMBLE_PIN, LOW);
  pinMode(RUMBLE_PIN, OUTPUT);

  #ifdef USE_ENCODER
  CalStick();
  #endif

  #ifdef USE_EEPROM
    //Start the I2C Library
    Wire.begin();
    Wire.setClock(I2C_CLOCK);
  #endif
  
  //Serial.println("Code has started!");
}

void ReadInputs(void)
{
    // clear it out
    memset(n64_buffer, 0, sizeof(n64_buffer));

    // buttons
    // First byte in n64_buffer should contain:
    // A, B, Z, Start, Dup, Ddown, Dleft, Dright
    bitWrite(n64_buffer[0], 7, !digitalRead(btn[0]));
    bitWrite(n64_buffer[0], 6, !digitalRead(btn[1]));
    bitWrite(n64_buffer[0], 5, !digitalRead(btn[2]));
    bitWrite(n64_buffer[0], 4, !digitalRead(btn[3]));
    bitWrite(n64_buffer[0], 3, !digitalRead(btn[4]));
    bitWrite(n64_buffer[0], 2, !digitalRead(btn[5]));
    bitWrite(n64_buffer[0], 1, !digitalRead(btn[6]));
    bitWrite(n64_buffer[0], 0, !digitalRead(btn[7]));
    
    // Second byte to N64 should contain:
    // Reset, 0, L, R, Cup, Cdown, Cleft, Cright
    //bitWrite(n64_buffer[1], 7, 0); // used below
    //bitWrite(n64_buffer[1], 6, 0); // unknown bit, padding?
    bitWrite(n64_buffer[1], 5, !digitalRead(btn[8]));
    bitWrite(n64_buffer[1], 4, !digitalRead(btn[9]));
    bitWrite(n64_buffer[1], 3, !digitalRead(btn[10]));
    bitWrite(n64_buffer[1], 2, !digitalRead(btn[11]));
    bitWrite(n64_buffer[1], 1, !digitalRead(btn[12]));
    bitWrite(n64_buffer[1], 0, !digitalRead(btn[13]));

    // user reset the controller
    byte l = bitRead(n64_buffer[1], 5);
    byte r = bitRead(n64_buffer[1], 4);
    byte start = bitRead(n64_buffer[0], 4);
    byte z = bitRead(n64_buffer[0], 5);
    if (l != 0 && r != 0)
    {
      if (start != 0)
      {
        //Serial.println("User reset the controller");
        bitWrite(n64_buffer[1], 7, 1); // set controller reset bit
        bitWrite(n64_buffer[0], 4, 0); // ignore start press
        CalStick();
      }
      #ifdef USE_EEPROM
      else if (z != 0)
      {
        long elapsed = lastSwitch - millis();
        if (elapsed > 500)
        {
          rumble_mode = !rumble_mode;
          lastSwitch = millis();
        }
      }
      #endif
    }

    // Third byte: Control Stick X position
    n64_buffer[2] = -zero_x + GetStick_x();
    // Fourth byte: Control Stick Y Position
    n64_buffer[3] = -zero_y + GetStick_y();

    // next 2 lines ignore joystick data, for testing 
    //n64_buffer[2] = 0;
    //n64_buffer[3] = 0;

    /*
    char buf[32];
    memset(buf, 0, 32);
    sprintf(buf, "0x%d%d%d%d%d%d%d%d%d%d%d%d%d%d",
      bitRead(n64_buffer[0], 7),
      bitRead(n64_buffer[0], 6),
      bitRead(n64_buffer[0], 5),
      bitRead(n64_buffer[0], 4),
      bitRead(n64_buffer[0], 3),
      bitRead(n64_buffer[0], 2),
      bitRead(n64_buffer[0], 1),
      bitRead(n64_buffer[0], 0),
      bitRead(n64_buffer[1], 5),
      bitRead(n64_buffer[1], 4),
      bitRead(n64_buffer[1], 3),
      bitRead(n64_buffer[1], 2),
      bitRead(n64_buffer[1], 1),
      bitRead(n64_buffer[1], 0)
    );
    Serial.print(buf);
    Serial.print(" ");
    Serial.print(n64_buffer[2], HEX);
    Serial.print(" ");
    Serial.print(n64_buffer[3], HEX);
    Serial.print(" ");
    Serial.println(rumble, HEX);
    */
}

void CalStick(void)
{
  //Serial.println("Calibrating analog stick...");
  #ifdef USE_ENCODER
    zero_x = 0;
    zero_y = 0;
  #else
    int t = 5;
    int x = 0;
    int y = 0;
    for (int i = 0; i < t; i++)
    {
      x += analogRead(JOY_X);
      y += analogRead(JOY_Y);
    }
    
    int center_x = x/t;
    int center_y = y/t;
    
    JOY_X_MIN = constrain(center_x-JOY_RANGE, 0, 1023);
    JOY_X_MAX = constrain(center_x+JOY_RANGE, 0, 1023);
    JOY_Y_MIN = constrain(center_y-JOY_RANGE, 0, 1023);
    JOY_Y_MAX = constrain(center_y+JOY_RANGE, 0, 1023);
  
    zero_x = GetStick_x();
    zero_y = GetStick_y();
  #endif

  /*
  Serial.print("Center x: ");
  Serial.println(center_x);
  Serial.print("Center y: ");
  Serial.println(center_y);
  Serial.println("Calibration complete!");
  */
}

#ifdef USE_ENCODER

void handleEncoderX()
{
  if(digitalRead(encoderIx) == digitalRead(encoderQx))
  {
    countx++;
  }
  else
  {
    countx--;
  }
  countx = constrain(countx, -JOY_MAX_REPORT, JOY_MAX_REPORT);
}

void handleEncoderY()
{
  if(digitalRead(encoderIy) == digitalRead(encoderQy))
  {
    county++;
  }
  else
  {
    county--;
  }
  county = constrain(county, -JOY_MAX_REPORT, JOY_MAX_REPORT);
}

#endif

signed int GetStick_x(void)
{
  #ifdef USE_ENCODER
    return countx;
  #else
    unsigned int l = analogRead(JOY_X);
    l = constrain(l, JOY_X_MIN, JOY_X_MAX);
    signed int i = map(l, JOY_X_MIN, JOY_X_MAX, -JOY_MAX_REPORT, JOY_MAX_REPORT);
    if (i < JOY_DEAD && i > -JOY_DEAD)
      return 0;
    return i;
  #endif
}

signed int GetStick_y(void)
{
  #ifdef USE_ENCODER
    return county;
  #else
    unsigned int l = analogRead(JOY_Y);
    l = constrain(l, JOY_Y_MIN, JOY_Y_MAX);
    signed int i = map(l, JOY_Y_MIN, JOY_Y_MAX, -JOY_MAX_REPORT, JOY_MAX_REPORT);
    if (i < JOY_DEAD && i > -JOY_DEAD)
      return 0;
    return i;
  #endif
}

void loop()
{
    int status;
    unsigned char data, addr;

    // control rumble motor
    analogWrite(RUMBLE_PIN, (rumble?RUMBLE_FORCE:0));

    ReadInputs();

    // Wait for incomming 64 command
    // this will block until the N64 sends us a command
    noInterrupts();
    get_n64_command();

    // 0x00 is identify command
    // 0x01 is status
    // 0x02 is read
    // 0x03 is write
    // 0xFF is reset and identify
    //
    // More info on reading and writing can be found here:
    // http://ultra64.ca/files/documentation/online-manuals/man/n64man/misc/glossarySystem.html#:~:text=Each%20256%20bytes%20is%20called,system%20for%20game%20note%20management.
    // and here:
    // http://n64devkit.square7.ch/pro-man/pro26/26-03.htm
    // and here:
    // http://hcs64.com/files/n64-hw.dox
    // and here:
    // https://github.com/sanni/cartreader/blob/b7dd3866700ca299e015ce86dfabd027bb4d17fc/Cart_Reader/N64.ino#L1367
    //
    
    switch (n64_command)
    {
        case 0xFF:
            CalStick();
            rumble = false;
        case 0x00:
            // identify
            // mutilate the n64_buffer array with our status
            // we return 0x050001 to indicate we have a rumble pack
            // or 0x050002 to indicate the expansion slot is empty
            //
            // 0xFF I've seen sent from Mario 64 and Shadows of the Empire.
            // I don't know why it's different, but the controllers seem to
            // send a set of status bytes afterwards the same as 0x00, and
            // it won't work without it.
            n64_buffer[0] = 0x05;
            n64_buffer[1] = 0x00;
            n64_buffer[2] = 0x01;

            n64_send(n64_buffer, 3, 0);

            //Serial.println("It was 0x00: an identify command");
            break;
        case 0x01:
            // blast out the pre-assembled array in n64_buffer
            n64_send(n64_buffer, 4, 0);

            //Serial.println("It was 0x01: the query command");
            break;
        case 0x02:
            // was the address the rumble latch at 0x8000?
            // decode the first half of the address, bits
            // 8 through 15
            addr = 0;
            addr |= (n64_raw_dump[0] != 0) << 7;
            addr |= (n64_raw_dump[1] != 0) << 6;
            addr |= (n64_raw_dump[2] != 0) << 5;
            addr |= (n64_raw_dump[3] != 0) << 4;
            addr |= (n64_raw_dump[4] != 0) << 3;
            addr |= (n64_raw_dump[5] != 0) << 2;
            addr |= (n64_raw_dump[6] != 0) << 1;
            addr |= (n64_raw_dump[7] != 0);
            
            // A read. If the address is 0x8000, return 32 bytes of 0x80 bytes,
            // and a CRC byte.  this tells the system our attached controller
            // pack is a rumble pack
            
            unsigned int myAddress = 0x0000;

            // Assume it's a read for 0x8000, which is the only thing it should
            // be requesting anyways (in rumble mode)
            if (addr == 0x8000) {
              memset(n64_buffer, rumble_mode?0x80:0x00, 32);
              n64_buffer[32] = 0xB8; // CRC
              n64_send(n64_buffer, 33, 1);
            }
            #ifdef USE_EEPROM
            else if (!rumble_mode)
            {
              myAddress |= (n64_raw_dump[0] != 0) << 15;
              myAddress |= (n64_raw_dump[1] != 0) << 14;
              myAddress |= (n64_raw_dump[2] != 0) << 13;
              myAddress |= (n64_raw_dump[3] != 0) << 12;
              myAddress |= (n64_raw_dump[4] != 0) << 11;
              myAddress |= (n64_raw_dump[5] != 0) << 10;
              myAddress |= (n64_raw_dump[6] != 0) << 9;
              myAddress |= (n64_raw_dump[7] != 0) << 8;
              myAddress |= (n64_raw_dump[8] != 0) << 7;
              myAddress |= (n64_raw_dump[9] != 0) << 6;
              myAddress |= (n64_raw_dump[10] != 0) << 5;
              myAddress |= (n64_raw_dump[11] != 0) << 4;
              myAddress |= (n64_raw_dump[12] != 0) << 3;
              myAddress |= (n64_raw_dump[13] != 0) << 2;
              myAddress |= (n64_raw_dump[14] != 0) << 1;
              myAddress |= (n64_raw_dump[15] != 0);

              readEEPROMPage(myAddress);
              memset(n64_buffer, 0x00, 32);
              for (long i = 0; i < 32; i++)
              {
                n64_buffer[i] = eepromData[i];
              }
              
              // TODO test this, not sure if the CRC is being calculated correctly here
              n64_buffer[32] = addrCRC(myAddress);
              n64_send(n64_buffer, 33, 1);
            }
            #endif

            /*
            Serial.println("It was 0x02: the read command");
            Serial.print("Addr was 0x");
            Serial.print(myAddress, HEX);
            Serial.println(" and data was:");
            for (int i = 0; i< 32; i++)
            {
              Serial.print("Addr:0x");
              Serial.print(myAddress+i, HEX);
              Serial.print(" data:0x");
              Serial.print(n64_buffer[i], HEX);
            }
            Serial.print(" and crc was 0x");
            Serial.print(n64_buffer[32], HEX);
            */
            break;
        case 0x03:
            // A write. we at least need to respond with a single CRC byte.  If
            // the write was to address 0xC000 and the data was 0x01, turn on
            // rumble! All other write addresses are ignored. (but we still
            // need to return a CRC)

            // decode the first data byte (fourth overall byte), bits indexed
            // at 24 through 31
            data = 0;
            data |= (n64_raw_dump[16] != 0) << 7;
            data |= (n64_raw_dump[17] != 0) << 6;
            data |= (n64_raw_dump[18] != 0) << 5;
            data |= (n64_raw_dump[19] != 0) << 4;
            data |= (n64_raw_dump[20] != 0) << 3;
            data |= (n64_raw_dump[21] != 0) << 2;
            data |= (n64_raw_dump[22] != 0) << 1;
            data |= (n64_raw_dump[23] != 0);

            // get crc byte, invert it, as per the protocol for
            // having a memory card attached
            n64_buffer[0] = crc_repeating_table[data] ^ 0xFF;

            // send it
            n64_send(n64_buffer, 1, 1);

            // end of time critical code
            
            // was the address the rumble latch at 0xC000?
            // decode the first half of the address, bits
            // 8 through 15
            addr = 0;
            addr |= (n64_raw_dump[0] != 0) << 7;
            addr |= (n64_raw_dump[1] != 0) << 6;
            addr |= (n64_raw_dump[2] != 0) << 5;
            addr |= (n64_raw_dump[3] != 0) << 4;
            addr |= (n64_raw_dump[4] != 0) << 3;
            addr |= (n64_raw_dump[5] != 0) << 2;
            addr |= (n64_raw_dump[6] != 0) << 1;
            addr |= (n64_raw_dump[7] != 0);

            if (addr == 0xC0 && rumble_mode) {
                //Rumble pak writes:
                //To switch on the rumble pak motor, the N64 sends:
                //03 C0 1B 01 01 01 ...
                //This writes 01 to addresses starting at 0x4000.
                
                //To turn the motor back off, the N64 sends:
                //03 C0 1B 00 00 00 ...
                rumble = (data != 0);
            }
            #ifdef USE_EEPROM
            else
            {
              unsigned int myAddress = 0x0000;
              myAddress |= (n64_raw_dump[0] != 0) << 15;
              myAddress |= (n64_raw_dump[1] != 0) << 14;
              myAddress |= (n64_raw_dump[2] != 0) << 13;
              myAddress |= (n64_raw_dump[3] != 0) << 12;
              myAddress |= (n64_raw_dump[4] != 0) << 11;
              myAddress |= (n64_raw_dump[5] != 0) << 10;
              myAddress |= (n64_raw_dump[6] != 0) << 9;
              myAddress |= (n64_raw_dump[7] != 0) << 8;
              myAddress |= (n64_raw_dump[8] != 0) << 7;
              myAddress |= (n64_raw_dump[9] != 0) << 6;
              myAddress |= (n64_raw_dump[10] != 0) << 5;
              myAddress |= (n64_raw_dump[11] != 0) << 4;
              myAddress |= (n64_raw_dump[12] != 0) << 3;
              myAddress |= (n64_raw_dump[13] != 0) << 2;
              myAddress |= (n64_raw_dump[14] != 0) << 1;
              myAddress |= (n64_raw_dump[15] != 0);
              
              long currentSpot = 0;
              long timerReset = 0;
              byte counter = 0;

              long len = sizeof(n64_raw_dump);
              memset(tempStore, 0x00, I2C_PAGE); // clear tempStore
              
              for (int i = 16; i < len-8; i++) // first 16 bits are address, last 8 are crc
              {
                tempStore[counter++] = n64_raw_dump[i];

                if (counter == I2C_PAGE)
                {
                  //Once we've collected a page worth, go ahead and do 
                  //a page write operation
                  writeEEPROMPage(myAddress+currentSpot);
                  counter = 0; //Reset count
                  currentSpot += I2C_PAGE;
                }
              }
            }
            #endif
            /*
            Serial.println("It was 0x03: the write command");
            Serial.print("Addr was 0x");
            Serial.print(myAddress, HEX);
            Serial.println(" and data was:");
            for (int i = 0; i< 32; i++)
            {
              Serial.print("Addr:0x");
              Serial.print(myAddress+i, HEX);
              Serial.print(" data:0x");
              Serial.print(tempStore[i], HEX);
            }
            */
            break;
            
        case 0x04:
            //Serial.println("It was 0x04: the CRC error command received!!");
            break;
            
        default:
            //Serial.print(millis(), DEC);
            //Serial.println(" | Unknown command received!!");
            break;

    }

    interrupts();  
}

#ifdef USE_EEPROM
void writeEEPROMPage(long eeAddress)
{
  //Serial.print("Writing EEPROM page to address: 0x");
  //Serial.print(eeAddress, HEX);
  //Serial.print(" to 0x");
  //Serial.println(eeAddress+I2C_PAGE, HEX);
  Wire.beginTransmission(EEPROM_ADR);

  Wire.write((int)(eeAddress >> 8)); // MSB
  Wire.write((int)(eeAddress & 0xFF)); // LSB

  //Write bytes to EEPROM
  for (byte x = 0 ; x < I2C_PAGE ; x++)
    Wire.write(tempStore[x]); //Write the data

  Wire.endTransmission(); //Send stop condition

  memset(tempStore, 0x00, I2C_PAGE); // clear tempStore
}

void readEEPROMPage(long eeAddress)
{
  //Serial.print("Reading EEPROM page from address: 0x");
  //Serial.print(eeAddress, HEX);
  //Serial.print(" to 0x");
  //Serial.println(eeAddress+I2C_PAGE, HEX);
  memset(eepromData, 0x00, I2C_PAGE); // clear page
  unsigned char i=0;
  Wire.beginTransmission(EEPROM_ADR);
  Wire.write((int)(eeAddress >> 8));   // MSB
  Wire.write((int)(eeAddress & 0xFF)); // LSB
  Wire.endTransmission();
 
  Wire.requestFrom(EEPROM_ADR,I2C_PAGE);
 
  while(Wire.available())
    eepromData[i++] = Wire.read();
}
#endif

/**
  * Waits for an incomming signal on the N64 pin and reads the command,
  * and if necessary, any trailing bytes.
  * 0x00 is an identify request
  * 0x01 is a status request
  * 0x02 is a controller pack read
  * 0x03 is a controller pack write
  * 0xFF is a controller reset and identify request
  *
  * for 0x02 and 0x03, additional data is passed in after the command byte,
  * which is also read by this function.
  *
  * All data is raw dumped to the n64_raw_dump array, 1 bit per byte, except
  * for the command byte, which is placed all packed into n64_command
  */
static void get_n64_command(void)
{
    int bitcount;
    char *bitbin = n64_raw_dump;
    int idle_wait;

    n64_command = 0;

    bitcount = 8;

    // wait to make sure the line is idle before
    // we begin listening
    for (idle_wait=32; idle_wait>0; --idle_wait) {
        if (!N64_QUERY) {
            idle_wait = 32;
        }
    }

read_loop:
        // wait for the line to go low
        while (N64_QUERY){}

        // wait approx 2us and poll the line
        asm volatile (
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                );
        if (N64_QUERY)
            n64_command |= 0x01;

        --bitcount;
        if (bitcount == 0)
            goto read_more;

        n64_command <<= 1;

        // wait for line to go high again
        // I don't want this to execute if the loop is exiting, so
        // I couldn't use a traditional for-loop
        while (!N64_QUERY) {}
        goto read_loop;

read_more:
        switch (n64_command)
        {
            case (0x03):
                // write command
                // we expect a 2 byte address and 32 bytes of data
                bitcount = 272 + 1; // 34 bytes * 8 bits per byte
                //Serial.println("command is 0x03, write");
                break;
            case (0x02):
                // read command 0x02
                // we expect a 2 byte address
                bitcount = 16 + 1;
                //Serial.println("command is 0x02, read");
                break;
            case (0x00):
            case (0x01):
            case (0xFF):
            default:
                // get the last (stop) bit
                bitcount = 1;
                break;
        }

        // make sure the line is high. Hopefully we didn't already
        // miss the high-to-low transition
        while (!N64_QUERY) {}
read_loop2:
        // wait for the line to go low
        while (N64_QUERY){}

        // wait approx 2us and poll the line
        asm volatile (
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                );
        *bitbin = N64_QUERY;
        ++bitbin;
        --bitcount;
        if (bitcount == 0)
            return;

        // wait for line to go high again
        while (!N64_QUERY) {}
        goto read_loop2;
}

/**
 * This sends the given byte sequence to the n64
 * length must be at least 1
 * hardcoded for Arduino DIO 8
 */
static void n64_send(unsigned char *buffer, char length, bool wide_stop)
{
    asm volatile (";Starting N64 Send Routine");
    // Send these bytes
    char bits;
    
    // This routine is very carefully timed by examining the assembly output.
    // Do not change any statements, it could throw the timings off
    //
    // We get 16 cycles per microsecond, which should be plenty, but we need to
    // be conservative. Most assembly ops take 1 cycle, but a few take 2
    //
    // I use manually constructed for-loops out of gotos so I have more control
    // over the outputted assembly. I can insert nops where it was impossible
    // with a for loop
    
    asm volatile (";Starting outer for loop");
outer_loop:
    {
        asm volatile (";Starting inner for loop");
        bits=8;
inner_loop:
        {
            // Starting a bit, set the line low
            asm volatile (";Setting line to low");
            N64_LOW; // 1 op, 2 cycles

            asm volatile (";branching");
            if (*buffer >> 7) {
                asm volatile (";Bit is a 1");
                // 1 bit
                // remain low for 1us, then go high for 3us
                // nop block 1
                asm volatile ("nop\nnop\nnop\nnop\nnop\n");
                
                asm volatile (";Setting line to high");
                N64_HIGH;

                // nop block 2
                // we'll wait only 2us to sync up with both conditions
                // at the bottom of the if statement
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              );

            } else {
                asm volatile (";Bit is a 0");
                // 0 bit
                // remain low for 3us, then go high for 1us
                // nop block 3
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\n");

                asm volatile (";Setting line to high");
                N64_HIGH;

                // wait for 1us
                asm volatile ("; end of conditional branch, need to wait 1us more before next bit");
                
            }
            // end of the if, the line is high and needs to remain
            // high for exactly 16 more cycles, regardless of the previous
            // branch path

            asm volatile (";finishing inner loop body");
            --bits;
            if (bits != 0) {
                // nop block 4
                // this block is why a for loop was impossible
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\n");
                // rotate bits
                asm volatile (";rotating out bits");
                *buffer <<= 1;

                goto inner_loop;
            } // fall out of inner loop
        }
        asm volatile (";continuing outer loop");
        // In this case: the inner loop exits and the outer loop iterates,
        // there are /exactly/ 16 cycles taken up by the necessary operations.
        // So no nops are needed here (that was lucky!)
        --length;
        if (length != 0) {
            ++buffer;
            goto outer_loop;
        } // fall out of outer loop
    }

    // send a single stop (1) bit
    // nop block 5
    asm volatile ("nop\nnop\nnop\nnop\n");
    N64_LOW;
    // wait 1 us, 16 cycles, then raise the line 
    // take another 3 off for the wide_stop check
    // 16-2-3=11
    // nop block 6
    asm volatile ("nop\nnop\nnop\nnop\nnop\n"
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\n");
    if (wide_stop) {
        asm volatile (";another 1us for extra wide stop bit\n"
                      "nop\nnop\nnop\nnop\nnop\n"
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\n");
    }

    N64_HIGH;

}

#ifdef USE_EEPROM
/******************************************
   N64 Controller CRC Functions
 *****************************************/
static word addrCRC(word address) {
  // CRC table
  word xor_table[16] = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x15, 0x1F, 0x0B, 0x16, 0x19, 0x07, 0x0E, 0x1C, 0x0D, 0x1A, 0x01 };
  word crc = 0;
  // Make sure we have a valid address
  address &= ~0x1F;
  // Go through each bit in the address, and if set, xor the right value into the output
  for (int i = 15; i >= 5; i--) {
    // Is this bit set?
    if ( ((address >> i) & 0x1)) {
      crc ^= xor_table[i];
    }
  }
  // Just in case
  crc &= 0x1F;
  // Create a new address with the CRC appended
  return address | crc;
}

/* unused
static byte dataCRC(byte * data) {
  byte ret = 0;
  for (byte i = 0; i <= 32; i++) {
    for (byte j = 7; j >= 0; j--) {
      int tmp = 0;
      if (ret & 0x80) {
        tmp = 0x85;
      }
      ret <<= 1;
      if ( i < 32 ) {
        if (data[i] & (0x01 << j)) {
          ret |= 0x1;
        }
      }
      ret ^= tmp;
    }
  }
  return ret;
}
*/
#endif

#if defined(USE_ENCODER) && defined(USE_EEPROM)
#  error "The atmega does not have enough pins to enable USE_ENCODER and USE_EEPROM simultaneously! The encoder makes use of the two I2C pins that are required by the EEPROM."
#endif
