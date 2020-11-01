/**
 * Gamecube controller to Nintendo 64 adapter
 * by Andrew Brown
 * https://github.com/brownan/Gamecube-N64-Controller/blob/master/gamecube.ino
 * 
 * 
 * Rewritten by jtryba so the atmega328 can be used as a stand alone replacement to the Nintendo CNT-NUS N64 controller chip, with built in rumble, for use in portables.
 * N64 controller protocol can be found here:
 * https://code.google.com/archive/p/micro-64-controller/wikis/Protocol.wiki
 */

/**
 * To use, hook up the following to the Arduino Atmega328
 * Digital I/O 8:  N64 serial line
 * Digital I/O 10: 220 ohm resistor to 2n2222 base, emitter to ground, collector to motor using 1N4007 flyback diode accross motor legs, and finally motor to 5v
 * All appropriate grounding and power lines, joystick, and all buttons (all active low, see pin definitions below)
 * 
 *   /------------\
 *  / O    O     O \
 * | GND Signl 3.3V |
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

 #if defined(ARDUINO_AVR_MINI)       
  #define BOARD "Mini"
#elif defined(ARDUINO_AVR_NANO)       
  #define BOARD "Nano"
#elif defined(ARDUINO_AVR_PRO)       
  #define BOARD "Pro"
#elif defined(ARDUINO_AVR_UNO)       
    #define BOARD "Uno"
#elif defined(ARDUINO_AVR_MICRO)       
    #define BOARD "Micro"
#else
   #error "Unsupported board"
#endif

//#define DEBUG
//#define DEBUG_VERBOSE
//#define USE_ENCODER 1 // uncomment this line to use the orig n64 encoder, only nano supports this

#ifdef DEBUG_VERBOSE
  #ifndef DEBUG
    #define DEBUG
  #endif
#endif

#include "pins_arduino.h"

#define N64_PIN 8
#define RUMBLE_PIN 10
#define N64_HIGH DDRB &= ~0x01
#define N64_LOW DDRB |= 0x01
#define N64_QUERY (PINB & 0x01)

// were using a PWM signal to control the rumble motor in order to save battery
#define RUMBLE_FORCE  250 //0-255

#define BUTTON_COUNT  14
#define JOY_DEAD      2
#define JOY_RANGE     400 // 1023 * 0.4 rounded a bit

//        Board: Mini/Uno // Nano
#ifdef USE_ENCODER
  
  #if defined(ARDUINO_AVR_MINI) || defined(ARDUINO_AVR_PRO) || defined(ARDUINO_AVR_MICRO)
    #error("This is board does not support n64 encoder, not enough pins!")
  #elif defined(ARDUINO_AVR_NANO)
    #define encoderIx A1 // 1
    #define encoderQx A2 // 4
    #define encoderIy A6 // 5
    #define encoderQy A7 // 6 (white)
  #elif defined(ARDUINO_AVR_UNO)
    #define encoderIx A1 // 1
    #define encoderQx A2 // 4
    #define encoderIy A4 // 5
    #define encoderQy A5 // 6 (white)
  #else
    #error("This is board does not support the encoder! Not enough pins, must use Arduino Nano.")
  #endif
  
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
#define BTN_A         0   // RX0
#define BTN_B         1   // TX1
#define BTN_C_UP      2   // D2
#define BTN_C_DOWN    3   // D3
#define BTN_C_LEFT    4   // D4
#define BTN_C_RIGHT   5   // D5
#define BTN_L         9   // D9
#define BTN_R         A0  // A0
#define BTN_Z         11  // D11
#define PAD_UP        6   // D6
#define PAD_DOWN      7   // D7
#define PAD_LEFT      12  // D12
#define PAD_RIGHT     A3  // A3
#define BTN_START     13  // D13

#ifdef USE_ENCODER
  void handleEncoderX(void);
  void handleEncoderX(void);
#endif

// Control sticks:
// 64 expects a signed value from -128 to 128 with 0 being neutral
//
// Additionally, the 64 controllers are relative. Whatever stick position
// it's in when it's powered on is what it reports as 0.
//
// While the joystick data is a signed 8 bit 2s complement we know from Micro
// that controllers only have 160 steps on them and I've had games which screw
// up when given the full 8 bit range.
//
// 160 steps diveded by 2 is 80 steps in each direction, adding a buffer i used 90
// also by using the controller_test.rom on official hardware, ive found that most controllers report closer to 90-100 anyway
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
static void n64_send_raw(unsigned char *buffer, char length);
void ReadInputs(void);
void CalStick(void);
signed int GetStick_x(void);
signed int GetStick_y(void);

static bool rumble = false;
static bool rumblelast = false;
static unsigned char enableRumble = 0x01;
static long lastSwitch = millis();

#include "crc_table.h"

void setup()
{
  // Communication with the N64 on this pin
  digitalWrite(N64_PIN, LOW);
  pinMode(N64_PIN, INPUT);

#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("------------------");
  Serial.println("Setup has started!");
#endif
  
  // setup I/O
  // buttons
  for (int i = 0; i < BUTTON_COUNT; i ++)
  {
#ifdef DEBUG
    if (btn[i] == 0 || btn[i] == 1)
    {
      continue;
    }
#endif
    digitalWrite(btn[i], LOW);
    pinMode(btn[i], INPUT_PULLUP);
  }
  
  // stick
  #ifdef USE_ENCODER
    countx=0;
    county=0;
    pinMode(encoderIx, INPUT);
    pinMode(encoderQx, INPUT);
    pinMode(encoderIy, INPUT);
    pinMode(encoderQy, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderIx), handleEncoderX, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderIy), handleEncoderY, CHANGE);
  #else
    pinMode(JOY_X, INPUT);
    pinMode(JOY_Y, INPUT);
  #endif
  
  // rumble
  digitalWrite(RUMBLE_PIN, LOW);
  pinMode(RUMBLE_PIN, OUTPUT);

  #ifndef USE_ENCODER
    CalStick();
  #endif

#ifdef DEBUG
  Serial.println("Code has started!");
#endif
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
    bitWrite(n64_buffer[1], 7, 0);
    bitWrite(n64_buffer[1], 6, 0);
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
        analogWrite(RUMBLE_PIN, RUMBLE_FORCE);
        delay(50);
        analogWrite(RUMBLE_PIN, 0);
        rumble = false;
        bitWrite(n64_buffer[1], 7, 1); // set controller reset flag
        bitWrite(n64_buffer[0], 4, 0); // ignore start press
        CalStick();
      }
      if (z != 0)
      {
        bitWrite(n64_buffer[0], 5, 0); // ignore z press
        long elapsed = millis() - lastSwitch;
        if (elapsed > 100)
        {
          //Serial.println("User toggled rumble");
          analogWrite(RUMBLE_PIN, RUMBLE_FORCE);
          delay(50);
          analogWrite(RUMBLE_PIN, 0);
          rumble = false;
          enableRumble++;
          if (enableRumble > 0x01)
          {
            enableRumble = 0x00;
            analogWrite(RUMBLE_PIN, RUMBLE_FORCE);
            delay(50);
            analogWrite(RUMBLE_PIN, 0);
          }
          lastSwitch = millis();
          identify();
        }
      }
    }

    // Third byte: Control Stick X position
    n64_buffer[2] = -zero_x + GetStick_x();
    // Fourth byte: Control Stick Y Position
    n64_buffer[3] = -zero_y + GetStick_y();
    
    n64_buffer[2] = 0;

#ifdef DEBUG_PAD_DATA
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
#endif
}

void CalStick(void)
{
#ifdef DEBUG
  Serial.println("Calibrating analog stick...");
#endif
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
#ifdef DEBUG
  Serial.print("Center x: ");
  Serial.println(center_x);
  Serial.print("Center y: ");
  Serial.println(center_y);
  Serial.println("Calibration complete!");
#endif
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

void identify()
{
  rumble = false;
            
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
  n64_buffer[2] = enableRumble; // = 0x01

  // 000001010000000000000001b

  uint8_t oldSREG = SREG;
  // Clear interrupts
  cli();
  n64_send(n64_buffer, 3, 0);
  // Restore old interrupt state
  SREG = oldSREG;
}

void loop()
{
    int status;
    unsigned char data, addr;

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
    //
    switch (n64_command)
    {
      // identify command(s)
        case 0x7F: // required to make pokemon puzzle league recognise micro
        case 0xFD: // required to make sm64 recognise micro
        case 0xFE:
        case 0xFF:
            //CalStick();
        case 0x00:
            identify();
            break;
      // query command
        case 0x01:
            // blast out the pre-assembled array in n64_buffer
            n64_send(n64_buffer, 4, 0);
            break;
      // read command
        case 0x02:
            // Addresses 8000-8FFF is used to query the enable state. 1 = enabled, 0 = disabled.
            
            // A read. If the address is 0x8000, return 32 bytes of 0x80 bytes,
            // and a CRC byte.  this tells the system our attached controller
            // pack is a rumble pack

            // Assume it's a read for 0x8000, which is the only thing it should
            // be requesting anyways
            memset(n64_buffer, (enableRumble == 0x01) ? 0x80 : 0x00, 32);
            n64_buffer[32] = 0xB8; // CRC 10111000b
            n64_send(n64_buffer, 33, 1);
            break;
      // write command
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

            if (addr == 0x80)
            {
              /*
              N64 sends: 03 80 01 followed by 32 bytes, all FE
              Response: 0xE1 with memory pak, 0x1E without.
              The response has no stop bit! instead, the data line
              goes low for 2us immediately after the last data bit. <- isnt that a long stop bit?
              N64 sends: 02 80 01
              Response: 32 0x80s with rumble pack, 32 0x00s without.
              Followed by CRC and no stop bit.
              */
              n64_buffer[1] = (enableRumble == 0x01) ? 0xE1 : 0x1E;
              n64_buffer[0] = crc_repeating_table[data] ^ 0xFF;
              n64_send_raw(n64_buffer, 2);
            }
            else
            {
              // get crc byte, invert it, as per the protocol for
              // having a memory card attached
              n64_buffer[0] = crc_repeating_table[data] ^ 0xFF;
              // send it
              n64_send(n64_buffer, 1, 1);
            }
            // end of time critical code

            // was the address the rumble latch at 0xC000?
            if (addr == 0xC0)
            {
                //Rumble pak writes:
                //To switch on the rumble pak motor, the N64 sends:
                //03 C0 1B 01 01 01 ...
                //This writes 01 to addresses starting at 0x4000.
                
                //To turn the motor back off, the N64 sends:
                //03 C0 1B 00 00 00 ...
                rumble = (data != 0);
            }
            
#ifdef DEBUG_VERBOSE
            Serial.print("Addr was 0x");
            Serial.print(addr, HEX);
            Serial.print(" and data was 0x");
            Serial.println(data, HEX);
#endif
            break;

        default:
        identify();
#ifdef DEBUG
            Serial.print(millis(), DEC);
            Serial.println(" | Unknown command received!!");
#endif
            break;

    }

    interrupts();

    if (rumble != rumblelast)
    {
      rumblelast = rumble;
      // control rumble motor
      analogWrite(RUMBLE_PIN, (rumble?RUMBLE_FORCE:0));
    }
    
#ifdef DEBUG_VERBOSE
    Serial.print("It was a 0x");
    Serial.print(n64_command, HEX);
    Serial.println(" command");
#endif
}

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
            case (0x7F):
            case (0xFD):
            case (0xFE):
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

/**
 * This sends the given byte sequence to the n64 without a stop bit
 * length must be at least 1
 * hardcoded for Arduino DIO 8
 */
static void n64_send_raw(unsigned char *buffer, char length)
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
    N64_HIGH;
}
