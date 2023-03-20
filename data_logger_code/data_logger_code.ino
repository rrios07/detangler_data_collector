#include <SPI.h>
#include <string.h>
#define EEPROM_ADDR 0x50 //101 0000
#define SPI_CLK 8000000 // 8 MHz SPI clock
#define ACC_CS_PIN 10
#define EEP_CS_PIN 9 //unsure of what value this is for now
#define AMP_SD_PIN A1
#define LED_PIN ? //need to define what this is
#define INTR_CNT 1552 //for 1 Hz with 1024 prescale and 16MHz clock
#define NUM_SAMPLES 16
#define DIV 4 //number of bit shifts for dividing by NUM_SAMPLES
#define DATA_LEN 7 //length of each chunk of data we are writing to eeprom in bytes
#define NUM_BLINKS 5 //how many times do we want startup LED to blink?
#define PAGE_BOUNDARY 256
#define MAX_AD 0x07ffff;

//struct for holding the 7 bytes of data per write
struct __attribute__ ((packed)) data_block {
  unsigned int tstamp : 8;
  int xAccel : 12;
  int yAccel : 12;
  int zAccel : 12;
  int strain : 12;
};

//settings for SPI for adxl363 accelerometer
SPISettings adxl363(SPI_CLK, MSBFIRST, SPI_MODE0);
SPISettings eeprom(SPI_CLK, MSBFIRST, SPI_MODE0);

uint8_t sample_fl = 0;
uint8_t wrap_fl = 0;
uint8_t sample_cnt = 0;
uin16_t max_accel = 0;
uint16_t max_strain = 0;
uint8_t timestamp = 0; //global timestamp var
uint32_t eeprom_addr = 0; //for addressing eeprom writes

struct data_block block; 

void setup() {
  // put your setup code here, to run once:

  pinMode(ACC_CS_PIN, OUTPUT);
  pinMode(EEP_CS_PIN, OUTPUT);
  pinMode(AMP_SD_PIN, OUTPUT);
  digitalWrite(AMP_SD_PIN, HIGH); //enable the amp
  SPI.Begin();

  //blink LED a NUM_BLINKS to indicate the device is powered on
  PinMode(LED_PIN, OUTPUT);
  for(int i = 0; i < NUM_BLINKS; i++){
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
  //setup timer interrupts
  cli(); //disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  //setup clock prescale to 1024 and CTC mode 
  TCCR1B |= (1 << WGM12) | (1 << CS12) | (1 << CS10); 
  OCR1A = INTR_CNT;//compare capture register value
  TIMSKI |= (1 << OCIE1A); //enable the interrupt
  sei(); //enable all interupts
}

/*
/ This is the interrupt service routine for Timer1. The timer is set to operate at
/ 20Hz, so there should be no timing issues with the length of the ISR. The job of this ISR
/ is to set a flag that allows the loop to pull.
*/
ISR(TIMER1_COMPA_vect){ 

  sample_fl = 1;
  sample_cnt = (sample_cnt % NUM_SAMPLES) + 1;
}

void loop() {
  // put your main code here, to run repeatedly:

  if(sample_fl == 1){ //one sample once the flag is set

    read_accel();
    read_strain();
    sample_fl = 0; //reset flag
  }

  //all 20 samples obtained; write to EEPROM
  if(sample_cnt == NUM_SAMPLES){
    write_data();
  }
}

//function to write accel data to the EEPROM
void write_data(){

  int i;
  int overshoot;
  uint8_t adr_l = eeprom_addr & 0xff;
  uint8_t adr_m = (eeprom_addr & 0xff00) >> 8;
  uint8_t adr_h = (eeprom_addr & 0x070000) >> 16;

  uint8_t sbuf[DATA_LEN];

  //sort out timestamp setup
  block.tstamp = timestamp;
  if(wrap_fl){
    block.stamp |= 0x80; //flip MSB 
  }

  memcpy(sbuf, (uint8_t*) block, DATA_LEN);

  //enable writing to EEPROM
  SPI.beginTransaction(eeprom);
  digitalWrite(EEP_CS_PIN, LOW); //set chip select low 
  SPI.transfer(0x06); //WE byte 
  digitalWrite(EEP_CS_PIN, HIGH); //set chip select high
  SPI.endTransaction(eemprom);

  //write data to EEPROM; 
  //page write the 5 bytes then increment the address and timestamp
  SPI.beginTransaction(eeprom);
  digitalWrite(EEP_CS_PIN, LOW); //set chip select low 
  SPI.transfer(0x02); //write request
  //send 3 address bytes
  SPI.transfer(adr_l);
  SPI.transfer(adr_m);
  SPI.transfer(adr_h);

  //send data
  for(i = 0; i < DATA_LEN; i++){
    SPI.transfer(sbuf[i]);
    eeprom_addr++;
  }
  digitalWrite(EEP_CS_PIN, HIGH); //set chip select high
  SPI.endTransaction(eemprom);

  //check address alignment with page and max addr
  overshoot = eeprom_addr % PAGE_BOUNDARY;
  if((overshoot < 7 && overshoot > 0){ //cross boundary condition
    eeprom_addr += (7 - overshoot);    
  }

  if(eeprom_addr > MAX_AD){ //wrap around condition
    eeprom_addr = 0;
    wrap_fl = ~wrap_fl;
  }

  timestamp = (timestamp + 1) % 60; //increment timestamp; [0,59]
}

//read the acceleration fron the 12-bit adc on the accelerometer
void read_strain(){

  uint16_t strain;
  uint16_t temp;

  SPI.beginTransaction(adxl363);
  digitalWrite(ACC_CS_PIN, LOW); //set chip select low 
  SPI.transfer(0x0B); //read request
  SPI.transfer(0x16); 
  strain = SPI.transfer(0x00); //lower 8 bits
  SPI.transfer(0x0B); //read request
  SPI.transfer(0x17); 
  temp = SPI.transfer(0x00);
  strain += (temp & 0xf) << 8; //upper 4 bits
  SPI.endTransaction(adxl363);
  digitalWrite(ACC_CS_PIN, HIGH); //set chip select high

  block.strain = strain & 0x0FFF;
}

//read acceleration values for x, y, and z via SPI interface. 
// This takes a total of 6 reads
void read_accel(){

  int16_t xtemp;
  int16_t ytemp;
  int16_t ztemp;
  int16_t temp;
  int16_t mag;

  //code to read from accelerometer SPI
  SPI.beginTransaction(adxl363);
  digitalWrite(ACC_CS_PIN, LOW); //set chip select low 
  
  //read accelerometer x, y, z
  SPI.transfer(0x0B); //read request
  SPI.transfer(0x0E); 
  xtemp = SPI.transfer(0x00); //lower 8 bits
  SPI.transfer(0x0B); //read request
  SPI.transfer(0x0F); 
  temp = SPI.transfer(0x00);
  xtemp += (temp & 0xf) << 8; //upper 4 bits

  SPI.transfer(0x0B); //read request
  SPI.transfer(0x10); 
  ytemp += SPI.transfer(0x00); //lower 8 bits
  SPI.transfer(0x0B); //read request
  SPI.transfer(0x11); 
  temp = SPI.transfer(0x00);
  ytemp += (temp & 0xf) << 8; //upper 4 bits

  SPI.transfer(0x0B); //read request
  SPI.transfer(0x12); 
  ztemp += SPI.transfer(0x00); //lower 8 bits
  SPI.transfer(0x0B); //read request
  SPI.transfer(0x13); 
  temp = SPI.transfer(0x00);
  ztemp += (temp & 0xf) << 8; //upper 4 bits

  SPI.endTransaction(adxl363);
  digitalWrite(ACC_CS_PIN, HIGH); //set chip select high

  block.xAccel = xtemp & 0x0fff;
  block.yAccel = ytemp & 0x0fff;
  block.zAccel = ztemp & 0x0fff;
}
