/*
 * This code is originally based on the work of Team Rhoban available at https://github.com/Rhoban/DXLBoard
 */
#ifndef DXL_PROTOCOL_HPP
#define DXL_PROTOCOL_HPP

#include "dxl.hpp"

#define DXL_PING        0x01
#define DXL_READ_DATA   0x02
#define DXL_WRITE_DATA  0x03
#define DXL_REG_WRITE   0x04
#define DXL_ACTION      0x05
#define DXL_RESET       0x06
#define DXL_REBOOT      0x08

#define DXL_STATUS      0x55

#define DXL_SYNC_READ   0x82
#define DXL_SYNC_WRITE  0x83
#define DXL_BULK_READ   0x92
#define DXL_BULK_WRITE  0x93

#define DXL_NO_ERROR    0x00

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
        unsigned short  modelNumber;
        unsigned char firmwareVersion;
        unsigned char id;
        unsigned char baudrate;
        unsigned char returnDelay;
        unsigned short cwLimit;
        unsigned short ccwLimit;
        unsigned char _dummy;
        unsigned char temperatureLimit;
        unsigned char lowestVoltage;
        unsigned char highestVoltage;
        unsigned short maxTorque;
        unsigned char returnStatus;
        unsigned char alarmLed;
        unsigned char alarmShutdown;
    } eeprom __attribute((packed));

    volatile unsigned char _dummy2[4]; 

    volatile struct dxl_ram {
        unsigned char torqueEnable;
        unsigned char led;
        unsigned char cwComplianceMargin;
        unsigned char ccwComplianceMargin;
        unsigned char cwComplianceSlope;
        unsigned char ccwComplianceSlope;
        unsigned short goalPosition;
        unsigned short movingSpeed;
        unsigned short torqueLimit;
        unsigned short presentPosition;
        unsigned short presentSpeed;
        unsigned short presentLoad;
        unsigned char presentVoltage;
        unsigned char presentTemperature;
        unsigned char registeredInstruction;
        unsigned char _dummy3;
        unsigned char moving;
        unsigned char lock;
        unsigned short punch;
    } ram __attribute__((packed));

    volatile char eeprom_dirty;
} __attribute__((packed));

#endif // DXL_PROTOCOL_HPP
