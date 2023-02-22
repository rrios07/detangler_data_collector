#include <wire.h>
#include <SPI.h>
#define EEPROM_ADDR 0x50 //101 0000
#define SPI_CLK 8000000 // 8 MHz SPI clock
#define CS_PIN 10
#define INTR_CNT 976 //for 16 Hz with 1024 prescale and 16MHz clock
#define NUM_SAMPLES 16
#define DIV 4 //number of bit shifts for dividing by NUM_SAMPLES
#define STRAIN_IN A1 //analog input A1 for strain gauge readings

//test comment
//settings for SPI for adxl363 accelerometer
SPISettings adxl363(SPI_CLK, MSBFIRST, SPI_MODE0);

uint8_t sample_fl = 0;
uint8_t sample_cnt = 0;
int32_t xval = 0;
int32_t yval = 0;
int32_t zval = 0;
uint16_t temp;

void setup() {
  // put your setup code here, to run once:

  Wire.begin();
  Wire.setClock(400000); //400kHz

  pinMode(CS_PIN, OUTPUT);
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

    uint8_t sv = read_strain();

    write_data();

    sample_fl = 0; //reset flag
  }

  //all 20 samples obtained
  if(sample_cnt == NUM_SAMPLES){
  
    //average the values for x, y, and z
    xval >>= DIV;
    yval >>= DIV;
    zval >>= DIV;

  }
}

//function to write accel data to the EEPROM
void write_data(){

  uint16_t mag = sqrt(xval * xval + yval * yval + zval * zval);


}

uint8_t read_strain(){
  uint8_t strain = analogRead(STRAIN_IN); //read strain from pin A1 (change if not using A1)
  return strain;  
}

//read acceleration values for x, y, and z via SPI interface. 
// This takes a total of 6 reads
void read_accel(){

    //code to read from accelerometer SPI
    SPI.beginTransaction(adxl363);
    digitalWrite(CS_PIN, LOW); //set chip select low 
    
    //read accelerometer x, y, z
    SPI.transfer(0x0B); //read request
    SPI.transfer(0x0E); //
    xval += SPI.transfer(0x00); //lower 8 bits
    SPI.transfer(0x0B); //read request
    SPI.transfer(0x0F); //
    temp = SPI.transfer(0x00);
    xval += (temp & 0xf) << 8; //upper 4 bits

    SPI.transfer(0x0B); //read request
    SPI.transfer(0x10); //
    yval += SPI.transfer(0x00); //lower 8 bits
    SPI.transfer(0x0B); //read request
    SPI.transfer(0x11); //
    temp = SPI.transfer(0x00);
    yval += (temp & 0xf) << 8; //upper 4 bits

    SPI.transfer(0x0B); //read request
    SPI.transfer(0x12); //
    zval += SPI.transfer(0x00); //lower 8 bits
    SPI.transfer(0x0B); //read request
    SPI.transfer(0x13); //
    temp = SPI.transfer(0x00);
    zval += (temp & 0xf) << 8; //upper 4 bits

    digitalWrite(CS_PIN, HIGH); //set chip select high
    SPI.endTransaction(adxl363);
  
}
