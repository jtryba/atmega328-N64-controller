/**
 * Gamecube controller to Nintendo 64 adapter
 * by Andrew Brown
 * Rewritten by jtryba so the atmega328 can be used as a stand alone replacement to the Nintendo CNT-NUS N64 controller chip, with built in rumble, for use in portables.
 * N64 controller protocol can be found here:
 * https://code.google.com/archive/p/micro-64-controller/wikis/Protocol.wiki
 */

/**
 * To use, hook up the following to the Arduino Atmega328
 * Digital I/O 8:  N64 serial line
 * Digital I/O 10: 820 ohm resistor to 2n2222 base, emitter to ground, collector to motor using 1N4007 flyback diode accross motor legs, and finally motor to 5v
 * All appropriate grounding and power lines, joystick, and all buttons (all active low, see pin definitions below)
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
 */

#include "pins_arduino.h"

#define N64_PIN 8
#define RUMBLE_PIN 10
#define N64_HIGH DDRB &= ~0x01
#define N64_LOW DDRB |= 0x01
#define N64_QUERY (PINB & 0x01)

// were using a PWM signal to control the rumble motor in order to save battery
#define RUMBLE_FORCE  128 //0-255

#define BUTTON_COUNT  14
#define JOY_DEAD      7
#define JOY_RANGE     500 // 1023/2 rounded a bit
#define JOY_X         A1
#define JOY_Y         A2
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
#define PAD_LEFT      A5
#define PAD_RIGHT     A3
#define BTN_START     A4

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
#define JOY_MAX_REPORT 90 // 127 for full 8 bit data

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

#include "crc_table.h"

void setup()
{
  Serial.begin(9600);

  Serial.println();
  Serial.println("Setup has started!");
  Serial.flush();

  // Status LED
  digitalWrite(13, LOW);
  pinMode(13, OUTPUT);

  // setup I/O
  // stick
  pinMode(JOY_X, INPUT);
  pinMode(JOY_Y, INPUT);
  
  // buttons
  for (int i = 0; i < BUTTON_COUNT; i ++)
  {
    digitalWrite(btn[i], LOW);
    pinMode(btn[i], INPUT_PULLUP);
  }
  
  // rumble
  digitalWrite(RUMBLE_PIN, LOW);
  pinMode(RUMBLE_PIN, OUTPUT);

  // Communication with the N64 on this pin
  digitalWrite(N64_PIN, LOW);
  pinMode(N64_PIN, INPUT);

  CalStick();
  
  Serial.println();
  Serial.println("Code has started!");
  Serial.flush();
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
    // 0, 0, L, R, Cup, Cdown, Cleft, Cright
    //bitWrite(n64_buffer[1], 7, 0);
    //bitWrite(n64_buffer[1], 6, 0);
    bitWrite(n64_buffer[1], 5, !digitalRead(btn[8]));
    bitWrite(n64_buffer[1], 4, !digitalRead(btn[9]));
    bitWrite(n64_buffer[1], 3, !digitalRead(btn[10]));
    bitWrite(n64_buffer[1], 2, !digitalRead(btn[11]));
    bitWrite(n64_buffer[1], 1, !digitalRead(btn[12]));
    bitWrite(n64_buffer[1], 0, !digitalRead(btn[13]));

    // Third byte: Control Stick X position
    n64_buffer[2] = -zero_x + GetStick_x();
    // Fourth byte: Control Stick Y Position
    n64_buffer[3] = -zero_y + GetStick_y();
}

void CalStick(void)
{
  Serial.println("Calibrating analog stick...");
  Serial.flush();
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
  
  JOY_X_MIN = center_x-JOY_RANGE;
  JOY_X_MAX = center_x+JOY_RANGE;
  JOY_Y_MIN = center_y-JOY_RANGE;
  JOY_Y_MAX = center_y+JOY_RANGE;

  zero_x = GetStick_x();
  zero_y = GetStick_y();

  Serial.println("Calibration complete!");
  Serial.flush();
}

signed int GetStick_x(void)
{    
  unsigned int l = analogRead(JOY_X);
  if (l > JOY_X_MAX)
    l = JOY_X_MAX;
  if (l < JOY_X_MIN)
    l = JOY_X_MIN;
  signed int i = map(l, JOY_X_MIN, JOY_X_MAX, -JOY_MAX_REPORT, JOY_MAX_REPORT);
  if (i < JOY_DEAD && i > -JOY_DEAD)
    return 0;
  return i;
}

signed int GetStick_y(void)
{
  unsigned int l = analogRead(JOY_Y);
  if (l > JOY_Y_MAX)
    l = JOY_Y_MAX;
  if (l < JOY_Y_MIN)
    l = JOY_Y_MIN;
  signed int i = map(l, JOY_Y_MIN, JOY_Y_MAX, -JOY_MAX_REPORT, JOY_MAX_REPORT);
  if (i < JOY_DEAD && i > -JOY_DEAD)
    return 0;
  return i;
}

static bool rumble = false;
void loop()
{
    int status;
    unsigned char data, addr;

    // control rumble motor
    //digitalWrite(RUMBLE_PIN, rumble);
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
    switch (n64_command)
    {
        case 0xFF:
          CalStick();
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
            // A read. If the address is 0x8000, return 32 bytes of 0x80 bytes,
            // and a CRC byte.  this tells the system our attached controller
            // pack is a rumble pack

            // Assume it's a read for 0x8000, which is the only thing it should
            // be requesting anyways
            memset(n64_buffer, 0x80, 32);
            n64_buffer[32] = 0xB8; // CRC

            n64_send(n64_buffer, 33, 1);

            //Serial.println("It was 0x02: the read command");
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

            if (addr == 0xC0) {
                //Rumble pak writes:
                //To switch on the rumble pak motor, the N64 sends:
                //03 C0 1B 01 01 01 ...
                //This writes 01 to addresses starting at 0x4000.
                //To turn the motor back off, the N64 sends:
                //03 C0 1B 00 00 00 ...
                rumble = (data != 0);
            }

            //Serial.println("It was 0x03: the write command");
            //Serial.print("Addr was 0x");
            //Serial.print(addr, HEX);
            //Serial.print(" and data was 0x");
            //Serial.println(data, HEX);
            break;

        default:
            Serial.print(millis(), DEC);
            Serial.println(" | Unknown command received!!");
            break;

    }

    interrupts();  
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
 * hardcoded for Arduino DIO 8 and external 2k-10k pull-up resistor
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
