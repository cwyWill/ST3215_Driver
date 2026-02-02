/*
 * INST.h
 * directive definition header file for waveshare serial bus servos.
 * date: 2023.6.28 
 */

#ifndef _INST_H
#define _INST_H

#include <cstddef>
#include <cstdint>

using s8 = int8_t;
using u8 = uint8_t;
using u16 = uint16_t;
using s16 = int16_t;
using u32 = uint32_t;
using s32 = int32_t;


enum class Instruction : u8{
    ping = 0x01,
    read = 0x02,
    write = 0x03,
    regWrite = 0x04,
    regAction = 0x05,
    syncRead = 0x82,
    syncWrite = 0x83,
    reset = 0x06,
    error,
};

enum class BaudRate : u8{
    r_1M = 0,
    r_500K = 1,
    r_250K = 2,
    r_128K = 3,
    r_115200 = 4,
    r_76800 = 5,
    r_57600 = 6,
    r_38400 = 7,
    r_19200 = 8,
    r_14400 = 9,
    r_9600 = 10,
    r_4800 = 11,
    r_maximum,
};

inline int getIntBaudrate( BaudRate br) {
    switch (br) {
        case BaudRate::r_1M   : return 1'000'000;
        case BaudRate::r_500K : return 500'000;
        case BaudRate::r_250K : return 250'000;
        case BaudRate::r_128K : return 128'000;
        case BaudRate::r_115200 : return 115200;
        case BaudRate::r_76800 : return 76800;
        case BaudRate::r_57600 : return 57600;
        case BaudRate::r_38400 : return 38400;
        case BaudRate::r_19200 : return 19200;
        case BaudRate::r_14400 : return 14400;
        case BaudRate::r_9600  : return 9600;
        case BaudRate::r_4800  : return 4800;
        default : return 0;
    }
}

enum class ErrorBit {
    error_voltage = 0b0000'0001,
    error_sensor = 0b0000'0010,
    error_temp = 0b0000'0100,
    error_curr = 0b0000'1000,
    error_angle = 0b0001'0000,
    error_overload = 0b0010'0000,
};

// #define INST_PING 0x01
// #define INST_READ 0x02
// #define INST_WRITE 0x03
// #define INST_REG_WRITE 0x04
// #define INST_REG_ACTION 0x05
// #define INST_SYNC_READ 0x82
// #define INST_SYNC_WRITE 0x83


// //波特率定义
// #define	_1M 0
// #define	_0_5M 1
// #define	_250K 2
// #define	_128K 3
// #define	_115200 4
// #define	_76800 5
// #define	_57600 6
// #define	_38400 7
// #define	_19200 8
// #define	_14400 9
// #define	_9600 10
// #define	_4800 11

#endif