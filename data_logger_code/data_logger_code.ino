#include <wire.h>
#include <SPI.h>
#define EEPROM_ADDR 0x50 //101 0000
#define SPI_CLK 8000000 // 8 MHz SPI clock
#define ACC_CS_PIN 10
#define EEP_CS_PIN 9 //unsure of what this is for now
#define INTR_CNT 976 //for 16 Hz with 1024 prescale and 16MHz clock
#define NUM_SAMPLES 16
#define DIV 4 //number of bit shifts for dividing by NUM_SAMPLES
#define DATA_LEN 5 //length of each chunk of data we are writing to eeprom in bytes

//test comment
//settings for SPI for adxl363 accelerometer
SPISettings adxl363(SPI_CLK, MSBFIRST, SPI_MODE0);
SPISettings eeprom(SPI_CLK, MSBFIRST, SPI_MODE0);

uint8_t sample_fl = 0;
uint8_t sample_cnt = 0;
uin16_t max_accel = 0;
uint16_t max_strain = 0;
uint8_t timestamp = 0;
uint32_t eeprom_addr = 0; //for addressing eeprom writes

void setup() {
  // put your setup code here, to run once:

  pinMode(ACC_CS_PIN, OUTPUT);
  pinMode(EEP_CS_PIN, OUTPUT);
  SPI.Begin();

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
/ is to set a flag that allows the loop to pull
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

  uint8_t adr_l = eeprom_addr & 0xff;
  uint8_t adr_m = (eeprom_addr & 0xff00) >> 8;
  uint8_t ard_h = (eeprom_addr & 0xff0000) >> 16;

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
  SPI.transfer(timestamp);
  SPI.transfer16(max_accel);
  SPI.transfer16(max_strain);
  digitalWrite(EEP_CS_PIN, HIGH); //set chip select high
  SPI.endTransaction(eemprom);

  eeprom_addr += DATA_LEN;  //increment address 
  timestamp = (timestamp + 1) % 60; //increment timestamp; [0,59]
  
  //reset variables for next pass
  cur_accel = 0;
  cur_strain = 0;
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


  if(strain > max_strain){
    max_strain = strain;
  }
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


  mag = sqrt(xtemp * xtemp + ytemp * ytemp + ztemp * ztemp);
  if(mag > max_accel){
    max_accel = mag;
  }
}
