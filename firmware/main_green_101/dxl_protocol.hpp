/*
 * This code is originally based on the work of Team Rhoban available at https://github.com/Rhoban/DXLBoard
 */
#ifndef DXL_PROTOCOL_HPP
#define DXL_PROTOCOL_HPP

#include "dxl.hpp"
#include <array>

constexpr unsigned int DXL_PING       = 0x01;
constexpr unsigned int DXL_READ_DATA  = 0x02;
constexpr unsigned int DXL_WRITE_DATA = 0x03;
constexpr unsigned int DXL_REG_WRITE  = 0x04;
constexpr unsigned int DXL_ACTION     = 0x05;
constexpr unsigned int DXL_RESET      = 0x06;
constexpr unsigned int DXL_REBOOT     = 0x08;

constexpr unsigned int DXL_STATUS     = 0x55;

constexpr unsigned int DXL_SYNC_READ  = 0x82;
constexpr unsigned int DXL_SYNC_WRITE = 0x83;
constexpr unsigned int DXL_BULK_READ  = 0x92;
constexpr unsigned int DXL_BULK_WRITE = 0x93;

constexpr unsigned int DXL_NO_ERROR   = 0x00;

void dxl_process(
    volatile struct dxl_device *device,
    volatile struct dxl_packet *packet,
    bool (*dxl_check_id)(volatile struct dxl_device *self, ui8 id),
    void (*dxl_write_data)(volatile struct dxl_device *self, ui8 id, ui8 addr, ui8 *values, ui8 length),
    void (*dxl_read_data)(volatile struct dxl_device *self, ui8 id, ui8 addr, ui8 *values, ui8 length, ui8 *error)
    );

struct dxl_registers
{
    volatile struct dxl_eeprom {
        uint16_t  modelNumber;
        unsigned char firmwareVersion;
        unsigned char id;
        unsigned char baudrate;
        unsigned char returnDelay;
        uint16_t cwLimit;
        uint16_t ccwLimit;
        unsigned char _dummy;
        unsigned char temperatureLimit;
        unsigned char lowestVoltage;
        unsigned char highestVoltage;
        uint16_t maxTorque;
        unsigned char returnStatus;
        unsigned char alarmLed;
        unsigned char alarmShutdown;
    } eeprom __attribute((packed));

    volatile std::array<unsigned char, 4> _dummy2;

    volatile struct dxl_ram {
        unsigned char torqueEnable;
        unsigned char led;
        unsigned char cwComplianceMargin;
        unsigned char ccwComplianceMargin;
        unsigned char cwComplianceSlope;
        unsigned char ccwComplianceSlope;
        uint16_t goalPosition;
        uint16_t movingSpeed;
        uint16_t torqueLimit;
        uint16_t presentPosition;
        uint16_t presentSpeed;
        uint16_t presentLoad;
        unsigned char presentVoltage;
        unsigned char presentTemperature;
        unsigned char registeredInstruction;
        unsigned char _dummy3;
        unsigned char moving;
        unsigned char lock;
        uint16_t punch;
    } ram __attribute__((packed));

    volatile char eeprom_dirty;
} __attribute__((packed));

#endif // DXL_PROTOCOL_HPP
