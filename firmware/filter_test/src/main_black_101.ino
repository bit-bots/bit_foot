#include <HardwareSerial.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <libmaple/usart.h>
#include <libmaple/gpio.h>
#include <wirish.h>
#include "malloc.h"
#include "dxl.h"
#include "ADS126X.h"
#include "BiQuadFilter.h"
#include <IIRFilter.h>
#include "Cascade.h"
#include "Plotter.h"
HardwareTimer adc_hw_timer(1);

HardwareTimer bus_hw_timer(2);


const double b_lp[] = { 0.00113722762905776, 0.00568613814528881, 0.0113722762905776,  0.0113722762905776,  0.00568613814528881, 0.00113722762905776 };
const double a_lp[] = { 1, -3.03124451613593, 3.92924380774061,  -2.65660499035499, 0.928185738776705, -0.133188755896548 };

IIRFilter lp0(b_lp, a_lp);
IIRFilter lp1(b_lp, a_lp);
IIRFilter lp2(b_lp, a_lp);
IIRFilter lp3(b_lp, a_lp);
IIRFilter filters[] = {lp0,lp1,lp2,lp3};
//IIRFilter highpass(b_hp, a_hp);
// start the class
ADS126X adc; 
//lowpassType *lp0, *lp1, *lp2, *lp3;
//lowpassType *filters[] = {lp0, lp1, lp2, lp3};

Plotter p;
// Arduino pin connected to CS on ADS126X
int chip_select = PA4; 

// positiv and negative reading pins for the 4 sensor
uint8_t pos_pin[4] = {7,5,3,1};
uint8_t neg_pin[4] = {6,4,2,0};

// current force values
int32_t force[4];
int32_t force_filtered[4];

// pin that is connected to the direction pin of the RS485 chip
int direction_pin = PB1;

// ID of this board 
ui8 DXL_ID = 101;

// buffer for dynamxiel bus
const int BUFFER_SIZE = 128;
ui8 write_buffer[BUFFER_SIZE];


volatile struct dxl_packet packet;
volatile struct dxl_packet ping_resp;
volatile struct dxl_packet read_resp;

// the sensor that is currently read
int current_sensor = 0;

uint64_t meassures = 0;
uint64_t times = 0;
uint64_t cycles = 0;
double f;


void bus_tick(){
  // read as many bytes as there are currently in the serial buffer
  int bytes_read = 0;
  uint32_t t1,t2;
  adc_hw_timer.pause();
  while(Serial1.available() > 0){
    ui8 inByte = Serial1.read();
    dxl_packet_push_byte(&packet, inByte);
    if(packet.process){
      // we recieved a complete package, check if it is for us
      if(packet.id == DXL_ID){
        //check if it is a ping or a read
        if(packet.instruction == 1){
          response_to_ping();
        }else if(packet.instruction == 2){

          response_to_read();
        }
      }
      packet.process = false;
      packet.dxl_state = 0;
      return;
    }
  }
  
  adc_hw_timer.resume();
}

void response_to_ping(){
  digitalWrite(direction_pin, HIGH);
  int len = dxl_write_packet(&ping_resp, write_buffer);
  Serial1.write(write_buffer, len);
  // make sure to write the complete buffer before changing direction pin  
  //delayMicroseconds(40);
  Serial1.flush();
  digitalWrite(direction_pin, LOW);
}

void response_to_read(){
  uint32_t t = micros();
  digitalWrite(direction_pin, HIGH);
  //make package with current bytes from the sensors

  //uint32_t t2 = micros();
  for(int i = 0; i < 4; i++){
    read_resp.parameters[i*4 + 0] = force_filtered[i]         & 0xFF;
    read_resp.parameters[i*4 + 1] = (force_filtered[i] >>  8) & 0xFF;
    read_resp.parameters[i*4 + 2] = (force_filtered[i] >> 16) & 0xFF;
    read_resp.parameters[i*4 + 3] = (force_filtered[i] >> 24) & 0xFF;
  }

  uint32_t t3 = micros();
  int len = dxl_write_packet(&read_resp, write_buffer);

  uint32_t t4 = micros();
  Serial1.write(write_buffer, len);

  uint32_t t5 = micros();
  // make sure to write the complete buffer before changing direction pin

  uint32_t t6 = micros();
  Serial1.flush();
  //delayMicroseconds(140);  

  uint32_t t7 = micros();
  digitalWrite(direction_pin, LOW);

  uint32_t t8 = micros();
  /*Serial.print(t2-t);
  Serial.print('\t');
  Serial.print(t3-t2);
  Serial.print('\t');
  Serial.print(t4-t3);
  Serial.print('\t');
  Serial.print(t5-t4);
  Serial.print('\t');
  Serial.print(t6-t5);
  Serial.print('\t');
  Serial.print(t7-t6);
  Serial.print('\t');
  Serial.print(t8-t7);
  Serial.print('\t');
  Serial.print(t8-t);
  Serial.print("\n");*/
}

void read_adc(void)
{
  meassures++;
  force[current_sensor] = adc.readADC1(pos_pin[current_sensor], neg_pin[current_sensor]);

  current_sensor = (current_sensor + 1) % 4;

  // after setting the pins we need to wait about 207 us for the adc to be ready again
  adc.setADC1Pins(pos_pin[current_sensor], neg_pin[current_sensor]);

  //bus_tick();

  // filter step
  force_filtered[current_sensor] = filters[current_sensor].filter(force[current_sensor]);
}

void loop(){
  bus_tick();
}

void setup() { 
  Serial.begin(2000000); 
  Serial1.begin(2000000); 

  pinMode(LED_BUILTIN, OUTPUT);    
  // adc reset pin
  pinMode(PA1, OUTPUT);
  digitalWrite(PA1, HIGH); 
  // adc start
  pinMode(PA3, OUTPUT);
  digitalWrite(PA3, HIGH); 
  

  pinMode(direction_pin, OUTPUT);
  digitalWrite(direction_pin, LOW);
  
  //create a ping resonse package which will always be the same
  dxl_packet_init(&packet);
  dxl_packet_init(&ping_resp);
  dxl_packet_init(&read_resp);

  ping_resp.id = DXL_ID;
  ping_resp.instruction = 0x55;
  // set to model number 5005 and firmware 1
  ping_resp.parameters[0] = 0x8D;
  ping_resp.parameters[1] = 0x13;
  ping_resp.parameters[2] = 0x01;
  ping_resp.parameter_nb = 3;

  read_resp.id = DXL_ID;
  read_resp.instruction = 0x55;
  read_resp.parameter_nb = 16;

  // adc setup
  adc.setStartPin(PA3);
  adc.begin(chip_select); // setup with chip select pin
  adc.setGain(ADS126X_GAIN_32);
  adc.setRate(ADS126X_RATE_38400);
  adc.enableInternalReference();
  adc.startADC1(); // start conversion on ADC1
  adc.disableStatus();
  adc.disableCheck();
  adc.setDelay(ADS126X_DELAY_0);
  adc.clearResetBit();  

  //adc sampling timer setup
  adc_hw_timer.pause();
  adc_hw_timer.setPeriod(500);
  adc_hw_timer.setCompare(TIMER_CH1, 1);
  adc_hw_timer.attachCompare1Interrupt(read_adc);
  adc_hw_timer.refresh();
  adc_hw_timer.resume();

  /*bus_hw_timer.pause();
  bus_hw_timer.setPeriod(1000);
  bus_hw_timer.attachInterrupt(2,bus_tick);
  bus_hw_timer.setCompare(TIMER_CH2, 1);
  bus_hw_timer.refresh();
  bus_hw_timer.resume();*/
}