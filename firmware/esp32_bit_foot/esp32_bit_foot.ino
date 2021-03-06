#include <array>
#include "ADS126X.h"
#include <Preferences.h>
#include <Dynamixel2Arduino.h>
#include "fast_slave.h"

// define two tasks for reading the dxl bus and doing other work
void TaskDXL( void *pvParameters );
void TaskWorker( void *pvParameters );
TaskHandle_t th_dxl,th_worker;

/*---------------------- DXL defines and variables ---------------------*/

#define DXL_DIR_PIN 22
#define DXL_PROTOCOL_VER_2_0 2.0
#define DXL_MODEL_NUM 0xaffe
#define DEFAULT_ID 101
#define DEFAULT_BAUD 4 //2mbaud since the flashing tool cant handle more

#define ADDR_CONTROL_ITEM_BAUD 8

#define ADDR_CONTROL_ITEM_SENSOR_0 36
#define ADDR_CONTROL_ITEM_SENSOR_1 40
#define ADDR_CONTROL_ITEM_SENSOR_2 44
#define ADDR_CONTROL_ITEM_SENSOR_3 48

// id and baud are stored using preferences for persistance between resets of the chip
Preferences prefs;
uint8_t id;
uint8_t baud;


uart_t* uart;


DYNAMIXEL::FastSlave dxl(NULL, DXL_MODEL_NUM);

/*---------------------- ADS defines and variables ---------------------*/
//ads variables 
ADS126X adc;
std::array<int32_t, 4> force; // read over the dxl bus

#define NUM_AVG 5 //number of samples to use for comparing the value to the median of the
#define ADS_CHIP_SELECT 5 // CS pin
#define ADS_RESET_PIN 4 


void setup() {
  disableCore0WDT(); // required since we dont want FreeRTOS to slow down our reading if the Wachdogtimer (WTD) fires
  disableCore1WDT();
  xTaskCreatePinnedToCore(
    TaskDXL
    ,  "TaskDXL"   // A name just for humans
    ,  65536  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority 3 since otherwise 
    ,  &th_dxl 
    ,  0);

  xTaskCreatePinnedToCore(
    TaskWorker
    ,  "TaskWork"
    ,  65536  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  &th_worker 
    ,  1);
}

void loop()
{
  // Tasks handle all the work
}

/*---------------------- DXL ------------------<---*/

uint32_t dxl_to_real_baud(uint8_t baud)
{
  int real_baud = 57600;
  switch(baud)
  {
    case 0: real_baud = 9600; break;
    case 1: real_baud = 57600; break;
    case 2: real_baud = 115200; break;
    case 3: real_baud = 1000000; break;
    case 4: real_baud = 2000000; break;
    case 5: real_baud = 3000000; break;
    case 6: real_baud = 4000000; break;
    case 7: real_baud = 4500000; break;
  }
  return real_baud;
}

void write_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void* arg)
{
  (void)dxl_err_code, (void)arg;
  if (item_addr == ADDR_CONTROL_ITEM_BAUD)
  {
    prefs.putUChar("baud", baud);
    ESP.restart(); // restart whole chip since restarting serial port crashes esp
  }
}

void TaskDXL(void *pvParameters) 
{
  (void) pvParameters;

  // enable pin
  pinMode(DXL_DIR_PIN, OUTPUT);

  
  prefs.begin("dxl");
  if(!prefs.getUChar("init")) // check if prefs are initialized
  {
    prefs.putUChar("id", DEFAULT_ID);
    prefs.putUChar("baud", DEFAULT_BAUD);
    prefs.putUChar("init",1); // set initialized
  }
  id = prefs.getUChar("id");
  baud = prefs.getUChar("baud");
  
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VER_2_0);
  dxl.setFirmwareVersion(1);
  dxl.setID(id);

  //dxl.addControlItem(ADDR_CONTROL_ITEM_BAUD, id); // not allowed since already implemented in library as default item
  dxl.addControlItem(ADDR_CONTROL_ITEM_BAUD, baud);
  dxl.addControlItem(ADDR_CONTROL_ITEM_SENSOR_0, force[0]);
  dxl.addControlItem(ADDR_CONTROL_ITEM_SENSOR_1, force[1]);
  dxl.addControlItem(ADDR_CONTROL_ITEM_SENSOR_2, force[2]);
  dxl.addControlItem(ADDR_CONTROL_ITEM_SENSOR_3, force[3]);
  dxl.setWriteCallbackFunc(write_callback_func);

  // init uart 0, given baud, 8bits 1stop no parity, pin 3 and 1, 256 buffer, no inversion
  uart = uartBegin(0, dxl_to_real_baud(baud), SERIAL_8N1, 3, 1,  256, false);
  // disable all interrupts
  uart->dev->conf1.rx_tout_en = 0;
  uart->dev->int_ena.rxfifo_full = 0;
  uart->dev->int_ena.frm_err = 0;
  uart->dev->int_ena.rxfifo_tout = 0;

  for (;;)
  {
    if(dxl.processPacket(uart)){
      if(dxl.getID() != id) // since we cant add the id as a control item, we need to check if it has been updated manually
      {
        id = dxl.getID();
        prefs.putUChar("id", id);
      }
    }
  }
}

/*--------------------- ADS ---------------------*/
void initADS()
{
  //reset ads1262
  digitalWrite(4, LOW);
  delayMicroseconds(2); // minimum of 4 t_clk = 0.54 us
  digitalWrite(4, HIGH);
  delayMicroseconds(4); // minimum of 8 t_clk = 1.09 us
  
  adc.begin(ADS_CHIP_SELECT); // setup with chip select pin
  adc.setGain(ADS126X_GAIN_32);
  adc.setRate(ADS126X_RATE_38400);
  adc.setFilter(ADS126X_SINC4);
  adc.enableInternalReference();
  adc.startADC1(); // start conversion on ADC1

  adc.disableStatus();
  adc.disableCheck();
  adc.setDelay(ADS126X_DELAY_0);
  adc.clearResetBit();
}

void TaskWorker(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  pinMode(4, OUTPUT);
  int current_sensor = 0;
  
  std::array<uint8_t, 4> pos_pin;
  std::array<uint8_t, 4> neg_pin;
  std::array<std::array<int32_t, NUM_AVG>, 4> previous_values;
  pos_pin[0]= 7;//ADS126X_AIN7;
  pos_pin[1]= 5;//ADS126X_AIN5;
  pos_pin[2]= 3;//ADS126X_AIN3;
  pos_pin[3]= 1;//ADS126X_AIN1;

  neg_pin[0]= 6;//ADS126X_AIN6;
  neg_pin[1]= 4;//ADS126X_AIN4;
  neg_pin[2]= 2;//ADS126X_AIN2;
  neg_pin[3]= 0;//ADS126X_AIN0;
  initADS();
  long count = 0;
  long time_counter = 0;
  
  for (;;)
  {
    // dummy read to change pin, see issue
    // https://github.com/Molorius/ADS126X/issues/5
    adc.readADC1(pos_pin[current_sensor], neg_pin[current_sensor]);
    delayMicroseconds(250);
    
    // actual read
    int32_t reading = adc.readADC1(pos_pin[current_sensor], neg_pin[current_sensor]);
    
    
    if(!reading || reading == 1) // detect if faulty reading occured
    {
      initADS();
    }
    else if (reading == 0x1000000) //detect if reset occured
    {
      if(adc.checkResetBit())
      {
        initADS();
      }
    }
    else
    {
      // sort to get median of previous values
      std::array<int32_t, NUM_AVG> current_previous = previous_values[current_sensor];
      std::sort(current_previous.begin(),current_previous.end());

      // check if current reading deviates too strongly from median
      if (abs(current_previous[NUM_AVG / 2] - reading) < 1e7)
      {
        force[current_sensor] = reading;
      }
      // update previous values for median calculation
      previous_values[current_sensor][count] = reading;

      // update current sensor
      current_sensor = (current_sensor + 1) % 4;

      // update counter for median calculating
      if(!current_sensor)
      {
        count = (count +1) % NUM_AVG;
      }
    }
  }
}
