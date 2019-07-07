/*
 * This code is originally based on the work of Team Rhoban available at https://github.com/Rhoban/DXLBoard
 */
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <wirish.h>
#include <libmaple/dma.h>
#include <libmaple/usart.h>
#include <libmaple/usb_cdcacm.h>
#include "dxl_protocol.h"
#include "dxl_serial.h"

// Serial com
#define DIRECTION1      29
#define DIRECTION2      30
#define DIRECTION3      31

// Devices allocation
static bool serialInitialized = false;
static uint8_t devicePorts[254];

// Device data
struct serial
{
    // Index
    int index;
    // Serial port
    HardwareSerial *port;
    // Direction pin
    int direction;
    // Transmission buffer
    char outputBuffer[DXL_BUFFER_SIZE];

    // Timestamp for sending data
    uint32_t packetSent;

    // DMA
    dma_tube_config tube_config;
    dma_channel channel;
    bool txComplete;
    bool dmaEvent;

    // master package buffer
    bool buffed_master_package;
    volatile struct dxl_packet *next_packet;
};

void usart_tcie(usart_reg_map *regs, int en) {
    if (en) regs->CR1 |= USART_CR1_TCIE;
    else regs->CR1 &= ~USART_CR1_TCIE;
}

// Ports that are online
struct serial *serials[8] = {0};

static void receiveMode(struct serial *serial);

ui8 status_send_buffer[DXL_BUFFER_SIZE];

static void serial_received(struct serial *serial)
{
    dma_disable(DMA1, serial->channel);
    usart_tcie(serial->port->c_dev()->regs, 0);
    serial->dmaEvent = false;
    serial->txComplete = true;
    receiveMode(serial);
}

static void tc_event(struct serial *serial)
{
    if (serial->dmaEvent) {
        serial_received(serial);
    }
}

static void dma_event(struct serial *serial)
{
    // DMA completed
    serial->dmaEvent = true;
}
static void DMAEvent1()
{
    dma_event(serials[1]);
}
static void usart1_tc()
{
    tc_event(serials[1]);
}
static void DMAEvent2()
{
    dma_event(serials[2]);
}
static void usart2_tc()
{
    tc_event(serials[2]);
}
static void DMAEvent3()
{
    dma_event(serials[3]);
}
static void usart3_tc()
{
    tc_event(serials[3]);
}

static void setupSerialDMA(struct serial *serial, int n)
{
    // We're receiving from the USART data register. serial->c_dev()
    // returns a pointer to the libmaple usart_dev for that serial
    // port, so this is a pointer to its data register.
    serial->tube_config.tube_dst = &serial->port->c_dev()->regs->DR;
    // We're only interested in the bottom 8 bits of that data register.
    serial->tube_config.tube_dst_size = DMA_SIZE_8BITS;
    // We're storing to rx_buf.
    serial->tube_config.tube_src = serial->outputBuffer;
    // rx_buf is a char array, and a "char" takes up 8 bits on STM32.
    serial->tube_config.tube_src_size = DMA_SIZE_8BITS;
    // Only fill BUF_SIZE - 1 characters, to leave a null byte at the end.
    serial->tube_config.tube_nr_xfers = n;
    // Flags:
    // - DMA_CFG_DST_INC so we start at the beginning of rx_buf and
    // fill towards the end.
    // - DMA_CFG_CIRC so we go back to the beginning and start over when
    // rx_buf fills up.
    // - DMA_CFG_CMPLT_IE to turn on interrupts on transfer completion.
    serial->tube_config.tube_flags = DMA_CFG_SRC_INC | DMA_CFG_CMPLT_IE | DMA_CFG_ERR_IE;
    // Target data: none. It's important to set this to NULL if you
    // don't have any special (microcontroller-specific) configuration
    // in mind, which we don't.
    serial->tube_config.target_data = NULL;
    // DMA request source.
    if (serial->index == 1) serial->tube_config.tube_req_src = DMA_REQ_SRC_USART1_TX;
    if (serial->index == 2) serial->tube_config.tube_req_src = DMA_REQ_SRC_USART2_TX;
    if (serial->index == 3) serial->tube_config.tube_req_src = DMA_REQ_SRC_USART3_TX;
}

static void receiveMode(struct serial *serial)
{
    asm volatile("nop");
    // Disabling transmitter
    //TODO serial->port->enableTransmitter(false);

    // Sets the direction to receiving
    digitalWrite(serial->direction, LOW);

    // Enabling the receiver
    //TODOserial->port->enableReceiver(true);
    asm volatile("nop");
}

void transmitMode(struct serial *serial)
{
    asm volatile("nop");
    // Disabling transmitter
    //TODOserial->port->enableReceiver(false);

    // Sets the direction to receiving
    digitalWrite(serial->direction, HIGH);

    // Enabling the receiver
    //TODOserial->port->enableTransmitter(true);
    asm volatile("nop");
}

void initSerial(struct serial *serial, int baudrate = DXL_DEFAULT_BAUDRATE)
{
    pinMode(serial->direction, OUTPUT);

    // Enabling serial port in receive mode
    serial->port->begin(baudrate);
    serial->port->c_dev()->regs->CR3 = USART_CR3_DMAT;
    
    // Entering receive mode
    receiveMode(serial);

    // Registering the serial port
    serials[serial->index] = serial;
}

void sendSerialPacket(struct serial *serial, volatile struct dxl_packet *packet)
{
    // We have a packet for the serial bus
    // First, clear the serial input buffers
    serial->port->flush();

    // Writing the packet in the buffer
    int n = dxl_write_packet(packet, (ui8 *)serial->outputBuffer);

    // Go in transmit mode
    transmitMode(serial);

#if 1
    // Then runs the DMA transfer
    serial->txComplete = false;
    serial->dmaEvent = false;
    setupSerialDMA(serial, n);
    dma_tube_cfg(DMA1, serial->channel, &serial->tube_config);
    dma_set_priority(DMA1, serial->channel, DMA_PRIORITY_VERY_HIGH);
    if (serial->index == 1) dma_attach_interrupt(DMA1, serial->channel, DMAEvent1);
    if (serial->index == 2) dma_attach_interrupt(DMA1, serial->channel, DMAEvent2);
    if (serial->index == 3) dma_attach_interrupt(DMA1, serial->channel, DMAEvent3);
    usart_tcie(serial->port->c_dev()->regs, 1);
    dma_enable(DMA1, serial->channel);
    serial->packetSent = millis();
#else
    // Directly send the packet
    char buffer[1024];
    n = dxl_write_packet(packet, (ui8 *)buffer);
    for (int i=0; i<n; i++) {
        serial->port->write(buffer[i]);
    }
    serial->port->waitDataToBeSent();
    receiveMode(serial);
#endif
}


/**
 * Ticking
 */
static void dxl_serial_tick(volatile struct dxl_device *self) 
{
    struct serial *serial = (struct serial*)self->data;

    // Timeout on sending packet, this should never happen
    if (!serial->txComplete && ((millis() - serial->packetSent) > 3)) {
        serial_received(serial);
    }
 
    if (serial->txComplete) {
        // send buffered master package
        if(serial->buffed_master_package){
            serial->buffed_master_package=false;
            sendSerialPacket(serial, serial->next_packet);
        }else{
            int n =0;
            // Just transmit all data on the serial bus to the usb bus
            while (serial->port->available() && !self->packet.process) {
                status_send_buffer[n] = serial->port->read();
                n++;
            }
            //SerialUSB.write(status_send_buffer, n);
        }
    }
}

/**
 * Processing master packets (sending it to the local bus)
 */
static void process(volatile struct dxl_device *self, volatile struct dxl_packet *packet)
{
    struct serial *serial = (struct serial*)self->data;
    dxl_serial_tick(self);

    // first check if this package is for the servos
    if(packet->id == DXL_BROADCAST || packet->id < 200){
        // write it if possible
        if (serial->txComplete) {
            self->packet.dxl_state = 0;
            self->packet.process = false;
            sendSerialPacket(serial, packet);
        }else{
            // we cant write it now, put it in a buffer. It will be send when DMA is finished
            serial->next_packet = packet;
            serial->buffed_master_package = true;
        }
    }

}

void dxl_serial_init(volatile struct dxl_device *device, int index)
{
    ASSERT(index >= 1 && index <= 3);

    // Initializing device
    struct serial *serial = (struct serial*)malloc(sizeof(struct serial));
    dxl_device_init(device);
    device->data = (void *)serial;
    device->tick = dxl_serial_tick;
    device->process = process;

    serial->index = index;
    serial->txComplete = true;
    serial->dmaEvent = false;

    if (index == 1) {
        serial->port = &Serial1;
        serial->direction = DIRECTION1;
        serial->channel = DMA_CH4;
    } else if (index == 2) {
        serial->port = &Serial2;
        serial->direction = DIRECTION2;
        serial->channel = DMA_CH7;
    } else {
        serial->port = &Serial3;
        serial->direction = DIRECTION3;
        serial->channel = DMA_CH2;
    }

    initSerial(serial);

    if (!serialInitialized) {
        serialInitialized = true;
    
        // Initializing DMA
        dma_init(DMA1);

        /*usart1_tc_handler = usart1_tc;
        usart2_tc_handler = usart2_tc;
        usart3_tc_handler = usart3_tc;*/

        // Reset allocation
        for (unsigned int k=0; k<sizeof(devicePorts); k++) {
            devicePorts[k] = 0;
        }
    }
    serial->buffed_master_package = false;
}
