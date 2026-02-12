/*
 * SMS_STS.h
 * application layer for waveshare ST servos.
 * date: 2023.6.11 
 */

#ifndef _SMS_STS_H
#define _SMS_STS_H

#include "SCSerial.h"
#include <string>
#include <utility>
#include <iostream>
#include <chrono>

using u8 = unsigned char;

//memory table definition
//-------EPROM(read only)--------
constexpr u8 SMS_STS_MODEL_L { 3 };
constexpr u8 SMS_STS_MODEL_H { 4 };


//-------EPROM(read & write)--------
constexpr u8 SMS_STS_ID { 5 };
constexpr u8 SMS_STS_BAUD_RATE { 6 };
constexpr u8 SMS_STS_RETURN_DELAY { 7 };
constexpr u8 SMS_STS_MIN_ANGLE_LIMIT_L { 9 };
constexpr u8 SMS_STS_MIN_ANGLE_LIMIT_H { 10 };
constexpr u8 SMS_STS_MAX_ANGLE_LIMIT_L { 11 };
constexpr u8 SMS_STS_MAX_ANGLE_LIMIT_H { 12 };
constexpr u8 SMS_STS_TEMPERATURE_LIMIT { 13 };
constexpr u8 SMS_STS_MAX_TORQUE { 16 };

constexpr u8 SMS_STS_POS_P_GAIN {21};
constexpr u8 SMS_STS_POS_D_GAIN {22};
constexpr u8 SMS_STS_POS_I_GAIN {23};

constexpr u8 SMS_STS_MIN_STARTUP_FORCE { 24 };
constexpr u8 SMS_STS_CW_DEAD { 26 };
constexpr u8 SMS_STS_CCW_DEAD { 27 };
constexpr u8 SMS_STS_OFS_L { 31 };
constexpr u8 SMS_STS_OFS_H { 32 };
constexpr u8 SMS_STS_MODE { 33 };
constexpr u8 SMS_STS_PROTECT_TORQUE { 34 };
constexpr u8 SMS_STS_SPEED_P_COEFF { 37 };
constexpr u8 SMS_STS_SPEED_I_COEFF { 39 };


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



enum class OperationMode {
	Position_servo,
	Speed,
	PWM,
	Step_servo,
};

struct MonitorParams {
	ReadStatus status {};
	double load {};
	double voltage {};
	int temperature {};
};

struct ServoFeedback {
	ReadStatus status {};
	std::chrono::steady_clock::time_point timestamp {};

	int position {};
	int speed {};
	double load {};
	double voltage {};
	int temperature {};

	bool asyncFlag {};
	u8 error {};
	bool isMoving {};
};


class SMS_STS : public SCSerial
{
// constructors
public:
	SMS_STS(std::string port);
	SMS_STS(std::string port, BaudRate rate);

// write command
public:
	// ! deprecated. Avoid to use
	virtual ReadResult<bool> writePosEx(int ID, s16 position, u16 speed, u8 acceleration = 0);//general write for single servo
	virtual ReadResult<bool> regWritePosEx(int ID, s16 position, u16 speed, u8 acceleration = 0);//position write asynchronously for single servo(call RegWriteAction to action)
	virtual void syncWritePosEx(int ID[], u8 IDN, s16 position[], u16 speed[], u8 acceleration[]);//write synchronously for multi servos

	// return bool: pure write
	// return int:  read data back
	

	// * Not recommended to change these parameters/attributes
	virtual ReadResult<bool> writeBaudrate(int ID, BaudRate rate);
	virtual ReadResult<bool> writeReturnDelay(int ID, u8 delay_2us);
	virtual ReadResult<bool> writeTemperatureLimit(int ID, u8 limit);
	virtual ReadResult<bool> writeMinStartForce(int ID, u16 force_permil);
	// clockwise, counterclockwise insensitive area
	virtual ReadResult<bool> writeMode(int ID, OperationMode mode);
	virtual ReadResult<bool> writeProtectiveTorque(int ID, u8 percent);
	virtual ReadResult<bool> writeSpeedPgain(int ID, u8 Pgain);
	virtual ReadResult<bool> writeSpeedIgain(int ID, u8 Igain);
	virtual ReadResult<bool> writeRunningtime(int ID, u16 runtime_permil);
	// virtual bool writeOverCurrentProtectTime(u8 ID, u8 time_10ms);

	// * Use carefully with only one servo on the bus
	virtual ReadResult<bool> writeID(int ID, u8 newID);

	// * Section where may be changed more often.
	virtual ReadResult<bool> writeMinAngle(int ID, u16 minAngle);
	virtual ReadResult<bool> writeMaxAngle(int ID, u16 maxAngle);
	virtual ReadResult<bool> resetMinMaxAngle(int ID);
	virtual ReadResult<bool> writeMultiCycle(int ID);
	virtual ReadResult<bool> writeTorqueLimit(int ID, u16 limit_permil);
	virtual ReadResult<bool> writePositionPID(int ID, u8 Pgain, u8 Igain, u8 Dgain);
	virtual ReadResult<bool> writePositionPgain(int ID, u8 Pgain);
	virtual ReadResult<bool> writePositionIgain(int ID, u8 Igain);
	virtual ReadResult<bool> writePositionDgain(int ID, u8 Dgain);
	virtual ReadResult<bool> writePositionCorrection(int ID, u16 correction, bool posDirection);
	virtual ReadResult<bool> writeAcceleration(int ID, u8 acc);
	virtual ReadResult<bool> writePosition(int ID, s16 position);
	virtual ReadResult<bool> writePositionRegister(int ID, s16 position);

	virtual ReadResult<bool> writeSpeed(int ID, s16 speed);//speed loop mode ctrl command

	virtual ReadResult<bool> enableTorque(int ID);//torque ctrl command
	virtual ReadResult<bool> disableTorque(int ID);//torque ctrl command
	virtual ReadResult<bool> unlockEEPROM(int ID);//eprom unlock
	virtual ReadResult<bool> lockEEPROM(int ID);//eprom locked
	virtual ReadResult<bool> offsetCalibration(int ID);//set middle position

// read information
public:
	// virtual ReadResult<bool> feedback(int ID);//servo information feedback
	virtual ReadResult<int> readPosition(int ID);//read position
	virtual ReadResult<int> readSpeed(int ID);//read speed
	virtual ReadResult<double> readLoad(int ID);//read motor load(0~1000, 1000 = 100% max load)
	virtual ReadResult<double> readVoltage(int ID);//read voltage
	virtual ReadResult<int> readTemp(int ID);//read temperature
	virtual ReadResult<double> readCurrent(int ID);//read current
	virtual ReadResult<OperationMode> readMode(int ID);//read working mode
	virtual ReadResult<bool> isMoving(int ID);//read move mode

	virtual ServoFeedback readFeedback(int ID);

public:
	constexpr static int s_broadcastID { 254 };

// member variables
private:
	u8 Mem[SMS_STS_PRESENT_CURRENT_H-SMS_STS_PRESENT_POSITION_L+1];

// protocol based constant
private:
	constexpr static double s_loadUnit { 0.1 };
	constexpr static double s_voltageUnit { 0.1 };
	constexpr static double s_currentUnit { 6.5 };
};

#endif	// _SMS_STS_H

