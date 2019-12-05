
#ifndef BIT_FOOT_CONFIG_HPP
#define BIT_FOOT_CONFIG_HPP

// Decide for witch foot the firmware should be false for Left true for Right.
#define LEFT_OR_RIGHT_FOOT 'r'

// Define DXL Bus ID
#define DXL_ID_RIGHT_FOOT 101
#define DXL_ID_LEFT_FOOT 102

// Which serial port should be use(1 or 3):
#define SERIAL_PORT 3


#if not (LEFT_OR_RIGHT_FOOT == 'l' || LEFT_OR_RIGHT_FOOT == 'r')
#error 'LEFT_OR_RIGHT_FOOT' is not defined as 'l' or 'r'!
#endif

#if not (SERIAL_PORT == 3 || SERIAL_PORT == 1)
#error 'SERIAL_PORT' is wrong defined, please chose correct serial port!
#endif

#endif //BIT_FOOT_CONFIG_HPP
