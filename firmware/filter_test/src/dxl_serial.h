/*
 * This code is originally based on the work of Team Rhoban available at https://github.com/Rhoban/DXLBoard
 */
#ifndef DXL_SERIAL_H
#define DXL_SERIAL_H

#include "dxl.h"

#define DXL_DEFAULT_BAUDRATE 2000000

void dxl_serial_init(volatile struct dxl_device *device, int index);
void usart_tcie(usart_reg_map *regs, int en);

#endif // DXL_SERIAL_H
