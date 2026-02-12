/*
 * SMS_STS.cpp
 * application layer for waveshare ST servos
 * date: 2023.6.17 
 */

#include "SMS_STS.h"
#include <iostream>
#include <cassert>
#include <vector>

std::ostream& operator<< (std::ostream& out, const MonitorParams& monitor) {
	if ( monitor.status == ReadStatus::Okay ) {
		out << " load: " << monitor.load << " %, voltage: "
						<< monitor.voltage << " V, temperature: "
						<< monitor.temperature << " C.\n";
		return out;
	}
	out << " motor info N/A\n";
	return out;
}

SMS_STS::SMS_STS(std::string port) : SMS_STS { port, BaudRate::r_1M }
{ }

SMS_STS::SMS_STS(std::string port, BaudRate rate) : SCSerial {port, rate}
{ }

ReadResult<bool> SMS_STS::writePosEx(int ID, s16 position, u16 speed, u8 acceleration)
{
	if(position<0){
		position = -position;
		position |= (1<<15);
	}
	u8 bBuf[7];
	bBuf[0] = acceleration;
	// put bytes into bBuf
	Host2SCS(bBuf+1, bBuf+2, position);
	Host2SCS(bBuf+3, bBuf+4, 0);
	Host2SCS(bBuf+5, bBuf+6, speed);
	
	// send message
	return genWrite(ID, SMS_STS_ACC, bBuf, 7);
}

ReadResult<bool> SMS_STS::regWritePosEx(int ID, s16 position, u16 speed, u8 acceleration)
{
	if(position<0){
		position = -position;
		position |= (1<<15);
	}
	u8 bBuf[7];
	bBuf[0] = acceleration;
	// put bytes into bBuf
	Host2SCS(bBuf+1, bBuf+2, position);
	Host2SCS(bBuf+3, bBuf+4, 0);
	Host2SCS(bBuf+5, bBuf+6, speed);
	
	// send message
	return regWrite(ID, SMS_STS_ACC, bBuf, 7);
}

// void SMS_STS::syncWritePosEx(u8 ID[], u8 IDN, s16 position[], u16 speed[], u8 acceleration[])
// {
//     u8 offbuf[7*IDN];
//     for(u8 i = 0; i<IDN; i++){
// 		if(position[i]<0){
// 			position[i] = -position[i];
// 			position[i] |= (1<<15);
// 		}
// 		u16 V;
// 		if(speed){
// 			V = speed[i];
// 		}else{
// 			V = 0;
// 		}
// 		if(acceleration){
// 			offbuf[i*7] = acceleration[i];
// 		}else{
// 			offbuf[i*7] = 0;
// 		}
//         Host2SCS(offbuf+i*7+1, offbuf+i*7+2, position[i]);
//         Host2SCS(offbuf+i*7+3, offbuf+i*7+4, 0);
//         Host2SCS(offbuf+i*7+5, offbuf+i*7+6, V);
//     }
//     syncWrite(ID, IDN, SMS_STS_ACC, offbuf, 7);
// }
void SMS_STS::syncWritePosEx(int ID[], u8 IDN, s16 position[], u16 speed[], u8 acceleration[])
{
    std::vector<u8> offbuf(7 * IDN);

    for (u8 i = 0; i < IDN; i++) {

        if (position[i] < 0) {
            position[i] = -position[i];
            position[i] |= (1 << 15);
        }

        u16 V = speed ? speed[i] : 0;

        offbuf[i*7] = acceleration ? acceleration[i] : 0;

        Host2SCS(offbuf.data() + i*7 + 1, offbuf.data() + i*7 + 2, position[i]);
        Host2SCS(offbuf.data() + i*7 + 3, offbuf.data() + i*7 + 4, 0);
        Host2SCS(offbuf.data() + i*7 + 5, offbuf.data() + i*7 + 6, V);
    }

    syncWrite(ID, IDN, SMS_STS_ACC, offbuf.data(), 7);
}
ReadResult<bool> SMS_STS::writeBaudrate(int ID, BaudRate rate) {
	return writeByte(ID, SMS_STS_BAUD_RATE, static_cast<u8>(rate));
}

ReadResult<bool> SMS_STS::writeReturnDelay(int ID, u8 delay_2us) {
	assert( delay_2us <= 254 && "Delay with 2us step can only range from 0 ~ 254");
	return writeByte(ID, SMS_STS_RETURN_DELAY, delay_2us);
}

ReadResult<bool> SMS_STS::writeTemperatureLimit(int ID, u8 limit) {
	assert( limit <= 100 && "Temperature limit should be with 100C.");
	return writeByte(ID, SMS_STS_TEMPERATURE_LIMIT, limit);
}

ReadResult<bool> SMS_STS::writeMinStartForce(int ID, u16 force_permil) {
	assert( force_permil <= 1000 && "Minimum startup force must be 0 ~ 100.0");
	return writeWord(ID, SMS_STS_MIN_STARTUP_FORCE, force_permil);
}

ReadResult<bool> SMS_STS::writeProtectiveTorque(int ID, u8 percent) {
	return writeByte(ID, SMS_STS_PROTECT_TORQUE, percent);
}

ReadResult<bool> SMS_STS::writeSpeedPgain(int ID, u8 Pgain) {
	return writeByte(ID, SMS_STS_SPEED_P_COEFF, Pgain);
}

ReadResult<bool> SMS_STS::writeSpeedIgain(int ID, u8 Igain) {
	return writeByte(ID, SMS_STS_SPEED_I_COEFF, Igain);
}

ReadResult<bool> SMS_STS::writeRunningtime(int ID, u16 runtime_permil) {
	return writeWord(ID, SMS_STS_GOAL_TIME_L, runtime_permil);
}


ReadResult<bool> SMS_STS::writeMode(int ID, OperationMode mode) {
	return writeByte(ID, SMS_STS_MODE, static_cast<u8>(mode));
}

ReadResult<bool> SMS_STS::writeID(int ID, u8 newID) {
	std::cout << "You are overwriting the ID of the motor from " << static_cast<int>(ID) << " to " << static_cast<int>(newID) << ". Press `y` to continue. Other keystroke will abort this process: ";
	char confirm {};
	std::cin >> confirm;
	if (confirm == 'y')
		return writeByte(ID, SMS_STS_ID, newID);
	std::cout << "ID reassignment aborted\n";
	return { false, ReadStatus::Abort, 0};
}


ReadResult<bool> SMS_STS::writeMinAngle(int ID, u16 minAngle) {
	assert( minAngle <= 4094 && "Min angle should be <= 4094");
	return writeWord(ID, SMS_STS_MIN_ANGLE_LIMIT_L, minAngle);
}

ReadResult<bool> SMS_STS::writeMaxAngle(int ID, u16 maxAngle) {
	assert( maxAngle <= 4095 && "Max angle should be <= 4095");
	return writeWord(ID, SMS_STS_MAX_ANGLE_LIMIT_L, maxAngle);
}


ReadResult<bool> SMS_STS::writeMultiCycle(int ID) {
	u8 msg[4] {0, 0, 0, 0};
	return genWrite(ID, SMS_STS_MIN_ANGLE_LIMIT_L, msg, 4);
}

ReadResult<bool> SMS_STS::resetMinMaxAngle(int ID) {
	u8 msg[4] {0, 0, 0xFF, 0x0F};
	return genWrite(ID, SMS_STS_MIN_ANGLE_LIMIT_L, msg, 4);
}

ReadResult<bool> SMS_STS::writeTorqueLimit(int ID, u16 limit_permil) {
	assert(limit_permil <= 1000 && "Torque limit should <= 1000.");
	return writeWord(ID, SMS_STS_MAX_TORQUE, limit_permil);
}

ReadResult<bool> SMS_STS::writePositionPID(int ID, u8 Pgain, u8 Igain, u8 Dgain) {
	u8 msg[3] {Pgain, Dgain, Igain};
	return genWrite(ID, SMS_STS_POS_P_GAIN, msg, 3);
}

ReadResult<bool> SMS_STS::writePositionPgain(int ID, u8 Pgain) {
	return writeByte(ID, SMS_STS_POS_P_GAIN, Pgain);
}

ReadResult<bool> SMS_STS::writePositionIgain(int ID, u8 Igain) {
	return writeByte(ID, SMS_STS_POS_I_GAIN, Igain);
}

ReadResult<bool> SMS_STS::writePositionDgain(int ID, u8 Dgain) {
	return writeByte(ID, SMS_STS_POS_D_GAIN, Dgain);
}

ReadResult<bool> SMS_STS::writePositionCorrection(int ID, u16 correction, bool posDirection) {
	assert( correction <= 2047 && "Correction value should be 0~2047");
	if (posDirection)
		correction |= ( 1 << 11);
	return writeWord(ID, SMS_STS_OFS_L, posDirection);
}

ReadResult<bool> SMS_STS::writePosition(int ID, s16 position) {
	if ( position < 0 ){
		position = -position;
		position |= (1<<15);
	}
	return writeWord(ID, SMS_STS_GOAL_POSITION_L, static_cast<u16>(position));
}

ReadResult<bool> SMS_STS::writePositionRegister(int ID, s16 position) {
	if ( position < 0 ) {
		position = -position;
		position |= (1<<15);
	}
	return writeWord(ID, SMS_STS_GOAL_POSITION_L, static_cast<u16>(position));
}


ReadResult<bool> SMS_STS::writeAcceleration(int ID, u8 acc) {
	return writeByte(ID, SMS_STS_ACC, acc);
}

ReadResult<bool> SMS_STS::writeSpeed(int ID, s16 speed)
{
	// Bit manipulation to satisfy protocol
	if( speed < 0 ){
		speed = -speed;
		speed |= (1<<15);
	}
	u8 bBuf[2];
	Host2SCS(bBuf+0, bBuf+1, speed);
	
	return genWrite(ID, SMS_STS_GOAL_SPEED_L, bBuf, 2);
}

ReadResult<bool> SMS_STS::enableTorque(int ID)
{
	return writeByte(ID, SMS_STS_TORQUE_ENABLE, 0x01);
}

ReadResult<bool> SMS_STS::disableTorque(int ID)
{
	return writeByte(ID, SMS_STS_TORQUE_ENABLE, 0x00);
}

ReadResult<bool> SMS_STS::unlockEEPROM(int ID)
{
	return writeByte(ID, SMS_STS_LOCK, 0);
}

ReadResult<bool> SMS_STS::lockEEPROM(int ID)
{
	return writeByte(ID, SMS_STS_LOCK, 1);
}

ReadResult<bool> SMS_STS::offsetCalibration(int ID)
{
	return writeByte(ID, SMS_STS_TORQUE_ENABLE, 128);
}

// ReadResult<bool> SMS_STS::feedback(int ID)
// {
// 	// int nLen = read(ID, SMS_STS_PRESENT_POSITION_L, Mem, sizeof(Mem));
// 	ReadResult result = read(ID, SMS_STS_PRESENT_POSITION_L, Mem, sizeof(Mem));
// 	return result.status;
// }

// TODO fix the code here
ReadResult<int> SMS_STS::readPosition(int ID)
{
	ReadResult<int> result = readWord(ID, SMS_STS_PRESENT_POSITION_L);
	if ( result.status != ReadStatus::Okay) {
		return result;
	}

	int pos { result.value };

	// Bit manipulation to satisfy the protocol
	if ( pos&(1<<15) ) {
		pos = -(pos&~(1<<15));
	}
	return { pos, ReadStatus::Okay };
}

// readSpeed. unit: step/s
ReadResult<int> SMS_STS::readSpeed(int ID) {
	ReadResult<int> result = readWord(ID, SMS_STS_PRESENT_SPEED_L);
	if ( result.status != ReadStatus::Okay) {
		return result;
	}

	int speed { result.value };

	// Bit manipulation to satisfy the protocol
	if ( speed&(1<<15) ) {
		speed = -(speed&~(1<<15));
	}	
	return { speed, ReadStatus::Okay };
}

ReadResult<double> SMS_STS::readLoad(int ID)
{
	ReadResult<int> result = readWord(ID, SMS_STS_PRESENT_LOAD_L);
	if (result.status != ReadStatus::Okay) {
		return {0., result.status};
	}
	int load { result.value };
	// Bit manipulation to satisfy the protocol
	if( load&(1<<10) ) {
		load = -(load&~(1<<10));
	}
	return { load * s_loadUnit, ReadStatus::Okay };
}

ReadResult<double> SMS_STS::readVoltage(int ID)
{	
	ReadResult<int> result = readByte(ID, SMS_STS_PRESENT_VOLTAGE);
	if ( result.status != ReadStatus::Okay ) {
		return {0., result.status };
	}
	int voltage { result.value };
	return { voltage * s_voltageUnit, ReadStatus::Okay };
}

ReadResult<int> SMS_STS::readTemp(int ID)
{	
	return readByte(ID, SMS_STS_PRESENT_TEMPERATURE);
}

ReadResult<bool> SMS_STS::isMoving(int ID)
{
	ReadResult<int> result = readByte(ID, SMS_STS_MOVING);
	int move { result.value };
	return { move == 1, result.status };
}

ReadResult<OperationMode> SMS_STS::readMode(int ID)
{
	ReadResult<int> result = readByte(ID, SMS_STS_MODE);
	OperationMode mode { static_cast<OperationMode>(result.value) };
	return {mode, result.status };
}

ReadResult<double> SMS_STS::readCurrent(int ID)
{
	ReadResult result = readWord(ID, SMS_STS_PRESENT_CURRENT_L);
	if ( result.status != ReadStatus::Okay) {
		return {0, result.status };
	}
	int current { result.value };
	
	// Bit manipulation to satisfy the protocol
	if ( current&(1<<15) ) {
		current = -(current&~(1<<15));
	}	
	return { current * s_currentUnit, ReadStatus::Okay };
}

ServoFeedback SMS_STS::readFeedback(int ID) {
	u16 position_r {};
	u16 speed_r {};
	u16 load_r {};
	u8 voltage_r {};
	u8 temperature_r {};

	u8 asyncFlag_r {};
	u8 error_r {};
	u8 isMoving_r {};

	u8 bBuf[11];
	ReadResult<int> result { read(ID, SMS_STS_PRESENT_POSITION_L, bBuf, 11) };
	if ( result.status != ReadStatus::Okay ) {
		std::cerr << "Fail to read feedback from motor ID: " << ID << '\n';
		return { result.status };	// other variables ignored
	}

	position_r = SCS2Host(bBuf[0], bBuf[1]);
	if ( position_r & ( 1<< 15) )
		position_r = -( position_r & ~(1<<15) );
	speed_r = SCS2Host(bBuf[2], bBuf[3]);
	if ( speed_r & ( 1<< 15) )
		speed_r = -( speed_r & ~(1<<15) );
	load_r = SCS2Host(bBuf[4], bBuf[5]);
	if ( load_r & (1<<10) ) {
		load_r = -( load_r & ~(1<<10) );
	}
	voltage_r = bBuf[6];
	temperature_r = bBuf[7];
	asyncFlag_r = bBuf[8];
	error_r = bBuf[9];
	isMoving_r = bBuf[10];

	return ServoFeedback {
		ReadStatus::Okay,
		std::chrono::steady_clock::now(),

		position_r,
		speed_r,
		load_r * s_loadUnit,
		voltage_r * s_voltageUnit,
		temperature_r,

		( asyncFlag_r == 1 ),
		error_r,
		( isMoving_r == 1 )
	};
}

