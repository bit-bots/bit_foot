#include "bit_foot_config.hpp"
#include "dxl.hpp"
#include "malloc.h"
#include <HardwareSerial.h>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <libmaple/gpio.h>
#include <libmaple/usart.h>
#include <wirish.h>

#include "ADS126X.h"

constexpr unsigned int MAGIC_NUM_0 = 0x55;
constexpr unsigned int MAGIC_NUM_1 = 0x8D;
constexpr unsigned int MAGIC_NUM_2 = 0x13;
constexpr unsigned int MAGIC_NUM_3 = 0x01;
constexpr unsigned int MAGIC_NUM_4 = 16;

// start the class
ADS126X adc;

// Arduino pin connected to CS on ADS126X
int chip_select = PA4;

// positiv and negative reading pins for the 4 sensor
std::array<uint8_t, 4> pos_pin;
std::array<uint8_t, 4> neg_pin;

// current force values
std::array<int32_t, 4> force;

// pin that is connected to the direction pin of the RS485 chip
int direction_pin = PB1;

// ID of this board
#if LEFT_OR_RIGHT_FOOT == 'r'
ui8 DXL_ID = DXL_ID_RIGHT_FOOT;
#elif LEFT_OR_RIGHT_FOOT == 'l'
ui8 DXL_ID = DXL_ID_LEFT_FOOT;
#endif

// buffer for dynamxiel bus
const int BUFFER_SIZE = 64;
ui8 write_buffer[BUFFER_SIZE];


volatile struct dxl_packet packet;
volatile struct dxl_packet ping_resp;
volatile struct dxl_packet read_resp;

// the sensor that is currently read
int current_sensor = 0;

//uint64_t startTime = millis();

//uint64_t meassures = 0;
//uint64_t time_sum = 0;
//uint64_t last_time = 0;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  // adc reset pin
  pinMode(PA1, OUTPUT);
  digitalWrite(PA1, HIGH);
  // adc start
  pinMode(PA3, OUTPUT);
  digitalWrite(PA3, HIGH);

#if SERIAL_PORT == 3
  Serial3.begin(2000000);
#elif SERIAL_PORT == 1
  Serial1.begin(2000000);
#endif

  pinMode(direction_pin, OUTPUT);
  digitalWrite(direction_pin, LOW);

  //create a ping resonse package which will always be the same
  dxl_packet_init(&packet);
  dxl_packet_init(&ping_resp);
  dxl_packet_init(&read_resp);

  ping_resp.id = DXL_ID;
  ping_resp.instruction = MAGIC_NUM_0;
  // set to model number 5005 and firmware 1
  ping_resp.parameters[0] = MAGIC_NUM_1;
  ping_resp.parameters[1] = MAGIC_NUM_2;
  ping_resp.parameters[2] = MAGIC_NUM_3;
  ping_resp.parameter_nb = 3;

  read_resp.id = DXL_ID;
  read_resp.instruction = MAGIC_NUM_0;
  read_resp.parameter_nb = MAGIC_NUM_4;


  pos_pin[0]= 7;//ADS126X_AIN7;
  pos_pin[1]= 5;//ADS126X_AIN5;
  pos_pin[2]= 3;//ADS126X_AIN3;
  pos_pin[3]= 1;//ADS126X_AIN1;

  neg_pin[0]= 6;//ADS126X_AIN6;
  neg_pin[1]= 4;//ADS126X_AIN4;
  neg_pin[2]= 2;//ADS126X_AIN2;
  neg_pin[3]= 0;//ADS126X_AIN0;

  adc.setStartPin(PA3);
  adc.begin(chip_select); // setup with chip select pin
  adc.setGain(ADS126X_GAIN_32);
  adc.setRate(ADS126X_RATE_7200);
  adc.setFilter(ADS126X_SINC4);
  adc.enableInternalReference();
  adc.startADC1(); // start conversion on ADC1

  adc.disableStatus();
  adc.disableCheck();
  adc.setDelay(ADS126X_DELAY_0);
  adc.clearResetBit();

}

void bus_tick(){
  // read as many bytes as there are currently in the serial buffer
#if SERIAL_PORT == 3
  while(Serial3.available() > 0){
    ui8 inByte = Serial3.read();
#elif SERIAL_PORT == 1
  while(Serial1.available() > 0){
    ui8 inByte = Serial1.read();
#endif
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
      return;
    }
  }
}


void response_to_ping(){
  digitalWrite(direction_pin, HIGH);
  unsigned int len = dxl_write_packet(&ping_resp, write_buffer);
#if SERIAL_PORT == 3
  Serial3.write(write_buffer, len);
#elif SERIAL_PORT == 1
  Serial1.write(write_buffer, len);
#endif
  // make sure to write the complete buffer before changing direction pin
  delayMicroseconds(40);
  digitalWrite(direction_pin, LOW);
}

void response_to_read(){
  digitalWrite(direction_pin, HIGH);
  //make package with current bytes from the sensors
  for(int i = 0; i < 4; i++){
    read_resp.parameters[i*4 + 0] = force[i]         & 0xFF;
    read_resp.parameters[i*4 + 1] = (force[i] >>  8) & 0xFF;
    read_resp.parameters[i*4 + 2] = (force[i] >> 16) & 0xFF;
    read_resp.parameters[i*4 + 3] = (force[i] >> 24) & 0xFF;
  }
  unsigned int len = dxl_write_packet(&read_resp, write_buffer);
#if SERIAL_PORT == 3
  Serial3.write(write_buffer, len);
#elif SERIAL_PORT == 1
  Serial1.write(write_buffer, len);
#endif
  // make sure to write the complete buffer before changing direction pin
  delayMicroseconds(140);
  //Serial1.flush();
  // out of some reasons flush doesn't work here

  digitalWrite(direction_pin, LOW);
}

void loop(){
  bus_tick();
  // dummy read to change pin, see issue
  // https://github.com/Molorius/ADS126X/issues/5
  adc.readADC1(pos_pin[current_sensor], neg_pin[current_sensor]);
  uint64_t current_time = micros();
  while(micros() - 1000 < current_time)
  {
      bus_tick();
  }
  // actual read
  force[current_sensor] = adc.readADC1(pos_pin[current_sensor], neg_pin[current_sensor]);
  current_sensor = (current_sensor + 1) % 4;
}


//void loop() {

//  bus_tick();

//  ADS_tick();


  /*long current_time = micros();
  time_sum += current_time - last_time;
  last_time = current_time;
  if (meassures > 1000){
      //Serial.println(time_sum / 1000.0);
      time_sum = 0;
      meassures = 0;
  }
  meassures++;*/

//}
