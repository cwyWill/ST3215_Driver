/*
 * SMS_STS.h
 * application layer for waveshare ST servos.
 * date: 2023.6.11 
 */

#ifndef _SMS_STS_H
#define _SMS_STS_H

#include <string>

using u8 = unsigned char;

//memory table definition
//-------EPROM(read only)--------
constexpr u8 SMS_STS_MODEL_L { 3 };
constexpr u8 SMS_STS_MODEL_H { 4 };


//-------EPROM(read & write)--------
constexpr u8 SMS_STS_ID { 5 };
constexpr u8 SMS_STS_BAUD_RATE { 6 };
constexpr u8 SMS_STS_MIN_ANGLE_LIMIT_L { 9 };
constexpr u8 SMS_STS_MIN_ANGLE_LIMIT_H { 10 };
constexpr u8 SMS_STS_MAX_ANGLE_LIMIT_L { 11 };
constexpr u8 SMS_STS_MAX_ANGLE_LIMIT_H { 12 };
constexpr u8 SMS_STS_CW_DEAD { 26 };
constexpr u8 SMS_STS_CCW_DEAD { 27 };
constexpr u8 SMS_STS_OFS_L { 31 };
constexpr u8 SMS_STS_OFS_H { 32 };
constexpr u8 SMS_STS_MODE { 33 };


//-------SRAM(read & write)--------
constexpr u8 SMS_STS_TORQUE_ENABLE { 40 };
constexpr u8 SMS_STS_ACC { 41 };
constexpr u8 SMS_STS_GOAL_POSITION_L { 42 };
constexpr u8 SMS_STS_GOAL_POSITION_H { 43 };
constexpr u8 SMS_STS_GOAL_TIME_L { 44 };
constexpr u8 SMS_STS_GOAL_TIME_H { 45 };
constexpr u8 SMS_STS_GOAL_SPEED_L { 46 };
constexpr u8 SMS_STS_GOAL_SPEED_H { 47 };
constexpr u8 SMS_STS_TORQUE_LIMIT_L { 48 };
constexpr u8 SMS_STS_TORQUE_LIMIT_H { 49 };
constexpr u8 SMS_STS_LOCK { 55 };


//-------SRAM(read only)--------
constexpr u8 SMS_STS_PRESENT_POSITION_L { 56 };
constexpr u8 SMS_STS_PRESENT_POSITION_H { 57 };
constexpr u8 SMS_STS_PRESENT_SPEED_L { 58 };
constexpr u8 SMS_STS_PRESENT_SPEED_H { 59 };
constexpr u8 SMS_STS_PRESENT_LOAD_L { 60 };
constexpr u8 SMS_STS_PRESENT_LOAD_H { 61 };
constexpr u8 SMS_STS_PRESENT_VOLTAGE { 62 };
constexpr u8 SMS_STS_PRESENT_TEMPERATURE { 63 };
constexpr u8 SMS_STS_MOVING { 66 };
constexpr u8 SMS_STS_PRESENT_CURRENT_L { 69 };
constexpr u8 SMS_STS_PRESENT_CURRENT_H { 70 };

#include "SCSerial.h"

class SMS_STS : public SCSerial
{
// constructors
public:
	SMS_STS(std::string port);
	SMS_STS(std::string port, BaudRate rate);

// write command
public:
	virtual int writePosEx(u8 ID, s16 position, u16 speed, u8 acceleration = 0);//general write for single servo
	virtual int regWritePosEx(u8 ID, s16 position, u16 speed, u8 acceleration = 0);//position write asynchronously for single servo(call RegWriteAction to action)
	virtual void syncWritePosEx(u8 ID[], u8 IDN, s16 position[], u16 speed[], u8 acceleration[]);//write synchronously for multi servos
	virtual int wheelMode(u8 ID);//speed loop mode
	virtual int writeSpeed(u8 ID, s16 speed, u8 acceleartion = 0);//speed loop mode ctrl command
	virtual int enableTorque(u8 ID);//torque ctrl command
	virtual int disableTorque(u8 ID);//torque ctrl command
	virtual int unlockEEPROM(u8 ID);//eprom unlock
	virtual int lockEEPROM(u8 ID);//eprom locked
	virtual int offsetCalibration(u8 ID);//set middle position

// read information
public:
	virtual int feedback(int ID);//servo information feedback
	virtual int readPosition(int ID);//read position
	virtual int readSpeed(int ID);//read speed
	virtual int readLoad(int ID);//read motor load(0~1000, 1000 = 100% max load)
	virtual int readVoltage(int ID);//read voltage
	virtual int readTemp(int ID);//read temperature
	virtual int readCurrent(int ID);//read current
	virtual int readMode(int ID);//read working mode
	virtual bool isMoving(int ID);//read move mode
private:
	u8 Mem[SMS_STS_PRESENT_CURRENT_H-SMS_STS_PRESENT_POSITION_L+1];
	bool m_status { false };

// getter and setter
public:
	bool getStatus() const { return m_status; }
};

#endif