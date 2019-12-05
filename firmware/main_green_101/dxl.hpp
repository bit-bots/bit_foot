/*
 * This code is originally based on the work of Team Rhoban available at https://github.com/Rhoban/DXLBoard
 */
#ifndef DXL_HPP
#define DXL_HPP

#include <cstdint>

// Protocol definition
constexpr unsigned int DXL_BROADCAST = 0xfe;

// Size limit for a buffer containing a dynamixel packet
constexpr unsigned int DXL_BUFFER_SIZE = 330;

// Maximum parameters in a packet
constexpr unsigned  int DXL_MAX_PARAMS = 240;


using ui8 = unsigned char;

/**
 * A dynamixel packet
 */
struct dxl_packet {
    ui8 id;
    ui8 instruction;
    ui8 error;
    ui8 parameter_nb;
    ui8 parameters[DXL_MAX_PARAMS];
    bool process; //is the package finished
    int dxl_state;
    int crc16;
};

void dxl_packet_init(volatile struct dxl_packet *packet);
void dxl_packet_push_byte(volatile struct dxl_packet *packet, ui8 b);
int dxl_write_packet(volatile struct dxl_packet *packet, ui8 *buffer);
void dxl_copy_packet(volatile struct dxl_packet *from, volatile struct dxl_packet *to);
uint16_t update_crc(uint16_t crc_accum, unsigned char *data_blk_ptr, uint16_t data_blk_size);

/**
 * A Dynamixel Device which is on the bus
 */
struct dxl_device
{
    void (*tick)(volatile struct dxl_device *self);
    void (*process)(volatile struct dxl_device *self, volatile struct dxl_packet *packet);
    volatile struct dxl_packet packet;
    volatile struct dxl_device *next;
    volatile void *data;
};

void dxl_device_init(volatile struct dxl_device *device);

/**
 * A bus is composed of one master and some slaves
 */
struct dxl_bus
{
    volatile struct dxl_device *master;
    volatile struct dxl_device *slaves;
};

/**
 * Initialize the bus and run its main loop
 */
void dxl_bus_init(struct dxl_bus *bus);
void dxl_bus_tick(struct dxl_bus *bus);

/**
 * Sets the master of add a slave on the bus
 */
void dxl_set_master(struct dxl_bus *bus, volatile struct dxl_device *master);
void dxl_add_slave(struct dxl_bus *bus, volatile struct dxl_device *slave);

#endif // DXL_HPP
