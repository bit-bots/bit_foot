/*
 * This code is originally based on the work of Team Rhoban available at https://github.com/Rhoban/DXLBoard
 */
#ifndef DXL_SERIAL_HPP
#define DXL_SERIAL_HPP

#include "dxl.hpp"

constexpr unsigned int DXL_DEFAULT_BAUDRATE = 2000000;

void dxl_serial_init(volatile struct dxl_device *device, int index);
void usart_tcie(usart_reg_map *regs, int en);

#endif // DXL_SERIAL_HPP
