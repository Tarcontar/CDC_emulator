#include "SPI.h"

static const uint16_t BYTES_DELAY = 874; //874 750
static const uint8_t PACKET_DELAY = 41;

#define BUFFER 10
volatile uint16_t captimehi = 0;
volatile uint16_t captimelo = 0;
volatile uint8_t capturingstart = 0;
volatile uint8_t capturingbytes = 0;
volatile uint32_t cmd [BUFFER];
volatile uint8_t cmd_write_buffer_pointer = 0;
volatile uint8_t cmd_read_buffer_pointer = 0;
volatile uint8_t cmdbit = 0;
volatile uint8_t newcmd [BUFFER];
volatile uint8_t prev_cmd = 0;

static const uint8_t CDC_PREFIX1 = 0x53;
static const uint8_t CDC_PREFIX2 = 0x2C;

static const uint8_t CDC_END_CMD = 0x14;
static const uint8_t CDC_END_CMD2 = 0x38;
static const uint8_t CDC_PLAY = 0xE4;
static const uint8_t CDC_STOP = 0x10;
static const uint8_t CDC_NEXT = 0xF8;
static const uint8_t CDC_PREV = 0x78;
static const uint8_t CDC_SEEK_FWD = 0xD8;
static const uint8_t CDC_SEEK_RWD = 0x58;
static const uint8_t cd1 = 0x0C;
static const uint8_t cd2 = 0x8C;
static const uint8_t cd3 = 0x4C;
static const uint8_t cd4 = 0xCC;
static const uint8_t cd5 = 0x2C;
static const uint8_t cd6 = 0xAC;
static const uint8_t CDC_SCAN = 0xA0;
static const uint8_t CDC_SFL = 0x60;
static const uint8_t CDC_PLAY_NORMAL = 0x08;

static const uint8_t MODE_PLAY = 0xFF;
static const uint8_t MODE_SHFFL = 0x55;
static const uint8_t MODE_SCAN = 0x00;

uint8_t cd = 0xBE;
uint8_t tr = 0xFE;
uint8_t mode = MODE_PLAY;

static const uint8_t DataOut = 2;

static const uint8_t PP = 9;
static const uint8_t VOL_PLUS = 8;
static const uint8_t VOL_MINUS = 7;
static const uint8_t NEXT = 6;
static const uint8_t PREV = 5;

uint8_t getCommand (uint32_t cmd2);
void read_Data_out ();
void initialize();
					//frame			cd			tr			time		time 		mode		frame 		frame
void send_package (uint8_t c0, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t c5, uint8_t c6, uint8_t c7);

void idle();
void pp();
void next();
void prev();
void vol_plus();
void vol_minus();

void setup()
{
	Serial.begin(9600);
	Serial.println("start");
	SPI.begin();

  pinMode (DataOut, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt(DataOut), read_Data_out, CHANGE);
  cli (); //stop interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1 = 0; // initialize counter value to 0
  // Set CS11 bit for 8 => tick every 1us @ 8MHz, 0.5us @ 16MHz
  // Set CS11 bit and CS10 for 64 prescaler => tick every 8us @ 8MHz, 4us @ 16MHz
  TCCR1B |= (1 << CS11); // | (1 << CS10);
  sei(); // allow interrupts
  
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE1);
	//SPI.setClockDivider(SPI_CLOCK_DIV128); // 62.5kHz@8MHz 125kHz @ 16MHz
	SPI.setClockDivider(SPI_CLOCK_DIV64); //125kHz@8MHz
	
	initialize();

  pinMode(PP, OUTPUT);
  digitalWrite(PP, LOW);
  pinMode(NEXT, OUTPUT);
  digitalWrite(NEXT, LOW);
  pinMode(PREV, OUTPUT);
  digitalWrite(PREV, LOW);
  pinMode(VOL_PLUS, OUTPUT);
  digitalWrite(VOL_PLUS, LOW);
  pinMode(VOL_MINUS, OUTPUT);
  digitalWrite(VOL_MINUS, LOW);

  while(1)
    idle();
}

void loop()
{ 
  idle();
}

void initialize()
{
	send_package (0x74, cd, tr, 0xFF, 0xFF, mode, 0x8F, 0x7C); // idle
	delay(10);
	send_package (0x34, 0xFF, tr, tr, tr, 0xFF, 0xFA, 0x3C); //load disc
	delay(100);
	send_package (0x74, cd, tr, 0xFF, 0xFF, mode, 0x8F, 0x7C); // idle
	delay(10);
}

void idle()
{
  send_package(0x34, cd, tr, 0xFF, 0xFF, 0xFF, 0xCF, 0x3C);
  delay(PACKET_DELAY);
}

void send_package(uint8_t c0, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t c5, uint8_t c6, uint8_t c7)
{
  uint8_t _buffer[8];
  for (int i = 0; i < 8; i++)
    _buffer[i] = 'A';
    
	_buffer[0] = SPI.transfer(c0);
	delayMicroseconds(BYTES_DELAY);
	_buffer[1] = SPI.transfer(c1);
	delayMicroseconds(BYTES_DELAY);
	_buffer[2] = SPI.transfer(c2);
	delayMicroseconds(BYTES_DELAY);
	_buffer[3] = SPI.transfer(c3);
	delayMicroseconds(BYTES_DELAY);
	_buffer[4] = SPI.transfer(c4);
	delayMicroseconds(BYTES_DELAY);
	_buffer[5] = SPI.transfer(c5);
	delayMicroseconds(BYTES_DELAY);
	_buffer[6] = SPI.transfer(c6);
	delayMicroseconds(BYTES_DELAY);
	_buffer[7] = SPI.transfer(c7);

  Serial.print(_buffer[0]); 
  Serial.print(" ");
  Serial.print(_buffer[1]); 
  Serial.print(" ");
  Serial.print(_buffer[2]);
  Serial.print(" ");
  Serial.print(_buffer[3]);
  Serial.print(" ");
  Serial.print(_buffer[4]);
  Serial.print(" ");
  Serial.print(_buffer[5]);
  Serial.print(" ");
  Serial.print(_buffer[6]);
  Serial.print(" ");
  Serial.print(_buffer[7]);
  Serial.print(" ");
  Serial.println();
}

uint8_t getCommand(uint32_t cmd2)
{
  if (((cmd2 >> 24) & 0xFF) == CDC_PREFIX1 && ((cmd2 >> 16) & 0xFF) == CDC_PREFIX2)
    if (((cmd2 >> 8) & 0xFF) == (0xFF ^ ((cmd2) & 0xFF)))
      return (cmd2 >> 8) & 0xFF;
  return 0;
}

void read_Data_out() // remote signals
{
  // if (newcmd == 0) {
  if (digitalRead(DataOut))
  {
    if (capturingstart || capturingbytes)
    {
      captimelo = TCNT1;
    }
    else
    {
      capturingstart = 1;
    }
    TCNT1 = 0;

    // eval times
    // tick @ 0.5us
    // 9000us HIGH and 4500us LOW
    if (captimehi > 17000 && captimelo > 8000) // && captimehi < 19000/8 && captimelo < 10000)
    {
      capturingstart = 0;
      capturingbytes = 1;
    }// Logic one = 1700us
    else if(capturingbytes && captimelo > 3300) // && captimelo < 3500)
    {
      //Serial.println("bit 1 ");
      cmd[cmd_write_buffer_pointer] = (cmd [cmd_write_buffer_pointer] << 1) | 0x00000001;
      cmdbit ++;
    }
    else if (capturingbytes && captimelo > 1000)// && captimelo < 1200)
    {
      //Serial.println("bit 0 ");
      cmd[cmd_write_buffer_pointer] = (cmd[cmd_write_buffer_pointer] << 1);
      cmdbit ++;
    }

    if (cmdbit == 32)
    {
      //Serial.println("32bit");
      //Serial.println(cmd[cmd_write_buffer_pointer], HEX);
      newcmd[cmd_write_buffer_pointer] = 1;
      capturingbytes = 0;
      cmdbit = 0;
      cmd_write_buffer_pointer ++;
      if (cmd_write_buffer_pointer == BUFFER)
        cmd_write_buffer_pointer = 0;
    }
  }
  else
  {
    captimehi = TCNT1;
    TCNT1 = 0;
  }
  //}
}

void pp()
{
  digitalWrite(PP, HIGH);
  delay(100);
  digitalWrite(PP, LOW);
}

void next()
{
  digitalWrite(NEXT, HIGH);
  delay(60); //60ms minimum!!!!
  digitalWrite(NEXT, LOW);
}

void prev()
{
  digitalWrite(PREV, HIGH);
  delay(200); //60ms minimum!!!!
  digitalWrite(PREV, LOW);
  delay(250);
  digitalWrite(PREV, HIGH);
  delay(200); //60ms minimum!!!!
  digitalWrite(PREV, LOW);
}

void vol_plus()
{
  digitalWrite(VOL_PLUS, HIGH);
  delay(500); //60ms minimum!!!!
  digitalWrite(VOL_PLUS, LOW);
}

void vol_minus()
{
  digitalWrite(VOL_MINUS, HIGH);
  delay(90); //60ms minimum!!!!
  digitalWrite(VOL_MINUS, LOW);
}




//messages from radio
/*
head head cmd  !cmd
0x53 0x2C 0xAA 0x55

switch on in cd mode/radio to cd (play)
0x53 0x2C 0xE4 0x1B
0x53 0x2C 0x14 0xEB

switch off in cd mode/cd to radio (pause)
0x53 0x2C 0x10 0xEF
0x53 0x2C 0x14 0xEB

next
0x53 0x2C 0xF8 0x7

prev
0x53 0x2C 0x78 0x87

seek next
0x53 0x2C 0xD8 0x27 hold down
0x53 0x2C 0xE4 0x1B release
0x53 0x2C 0x14 0xEB

seek prev
0x53 0x2C 0x58 0xA7 hold down
0x53 0x2C 0xE4 0x1B release
0x53 0x2C 0x14 0xEB

cd 1
0x53 0x2C 0x0C 0xF3
0x53 0x2C 0x14 0xEB
0x53 0x2C 0x38 0xC7
send new cd no. to confirm change, else:
0x53 0x2C 0xE4 0x1B beep, no cd (same as play)
0x53 0x2C 0x14 0xEB

cd 2
0x53 0x2C 0x8C 0x73
0x53 0x2C 0x14 0xEB
0x53 0x2C 0x38 0xC7
send new cd no. to confirm change, else:
0x53 0x2C 0xE4 0x1B beep, no cd (same as play)
0x53 0x2C 0x14 0xEB

cd 3
0x53 0x2C 0x4C 0xB3
0x53 0x2C 0x14 0xEB
0x53 0x2C 0x38 0xC7
send new cd no. to confirm change, else:
0x53 0x2C 0xE4 0x1B beep, no cd (same as play)
0x53 0x2C 0x14 0xEB

cd 4
0x53 0x2C 0xCC 0x33
0x53 0x2C 0x14 0xEB
0x53 0x2C 0x38 0xC7
send new cd no. to confirm change, else:
0x53 0x2C 0xE4 0x1B beep, no cd (same as play)
0x53 0x2C 0x14 0xEB

cd 5
0x53 0x2C 0x2C 0xD3
0x53 0x2C 0x14 0xEB
0x53 0x2C 0x38 0xC7
send new cd no. to confirm change, else:
0x53 0x2C 0xE4 0x1B beep, no cd (same as play)
0x53 0x2C 0x14 0xEB

cd 6
0x53 0x2C 0xAC 0x53
0x53 0x2C 0x14 0xEB
0x53 0x2C 0x38 0xC7
send new cd no. to confirm change, else:
0x53 0x2C 0xE4 0x1B beep, no cd (same as play)
0x53 0x2C 0x14 0xEB

scan (in 'play', 'shffl' or 'scan' mode)
0x53 0x2C 0xA0 0x5F

shuffle in 'play' mode
0x53 0x2C 0x60 0x9F

shuffle in 'shffl' mode
0x53 0x2C 0x08 0xF7
0x53 0x2C 0x14 0xEB

*/


 

