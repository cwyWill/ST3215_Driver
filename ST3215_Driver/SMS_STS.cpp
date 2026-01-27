/*
 * SMS_STS.cpp
 * application layer for waveshare ST servos
 * date: 2023.6.17 
 */

#include "SMS_STS.h"
#include <iostream>
#include <cassert>
#include <vector>

SMS_STS::SMS_STS(std::string port) : SMS_STS { port, BaudRate::r_1M }
{ }

SMS_STS::SMS_STS(std::string port, BaudRate rate) : SCSerial {port, rate}
{ }

int SMS_STS::writePosEx(u8 ID, s16 position, u16 speed, u8 acceleration)
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

int SMS_STS::regWritePosEx(u8 ID, s16 position, u16 speed, u8 acceleration)
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
void SMS_STS::syncWritePosEx(u8 ID[], u8 IDN, s16 position[], u16 speed[], u8 acceleration[])
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
bool SMS_STS::writeBaudrate(u8 ID, BaudRate rate) {
	return writeByte(ID, SMS_STS_BAUD_RATE, static_cast<u8>(rate));
}

bool SMS_STS::writeReturnDelay(u8 ID, u8 delay_2us) {
	assert( delay_2us <= 254 && "Delay with 2us step can only range from 0 ~ 254");
	return writeByte(ID, SMS_STS_RETURN_DELAY, delay_2us);
}

bool SMS_STS::writeTemperatureLimit(u8 ID, u8 limit) {
	assert( limit <= 100 && "Temperature limit should be with 100C.");
	return writeByte(ID, SMS_STS_TEMPERATURE_LIMIT, limit);
}

bool SMS_STS::writeMinStartForce(u8 ID, u16 force_permil) {
	assert( force_permil <= 1000 && "Minimum startup force must be 0 ~ 100.0");
	return writeWord(ID, SMS_STS_MIN_STARTUP_FORCE, force_permil);
}

bool SMS_STS::writeProtectiveTorque(u8 ID, u8 percent) {
	return writeByte(ID, SMS_STS_PROTECT_TORQUE, percent);
}

bool SMS_STS::writeSpeedPgain(u8 ID, u8 Pgain) {
	return writeByte(ID, SMS_STS_SPEED_P_COEFF, Pgain);
}

bool SMS_STS::writeSpeedIgain(u8 ID, u8 Igain) {
	return writeByte(ID, SMS_STS_SPEED_I_COEFF, Igain);
}

bool SMS_STS::writeRunningtime(u8 ID, u16 runtime_permil) {
	return writeWord(ID, SMS_STS_GOAL_TIME_L, runtime_permil);
}


bool SMS_STS::writeMode(u8 ID, OperationMode mode) {
	return writeByte(ID, SMS_STS_MODE, static_cast<u8>(mode));
}

bool SMS_STS::writeID(u8 ID, u8 newID) {
	std::cout << "You are overwriting the ID of the motor from " << static_cast<int>(ID) << " to " << static_cast<int>(newID) << ". Press `y` to continue. Other keystroke will abort this process: ";
	char confirm {};
	std::cin >> confirm;
	if (confirm == 'y')
		return writeByte(ID, SMS_STS_ID, newID);
	std::cout << "ID reassignment aborted\n";
	return false;
}


bool SMS_STS::writeMinAngle(u8 ID, u16 minAngle) {
	assert( minAngle <= 4094 && "Min angle should be <= 4094");
	return writeWord(ID, SMS_STS_MIN_ANGLE_LIMIT_L, minAngle);
}

bool SMS_STS::writeMaxAngle(u8 ID, u16 maxAngle) {
	assert( maxAngle <= 4095 && "Max angle should be <= 4095");
	return writeWord(ID, SMS_STS_MAX_ANGLE_LIMIT_L, maxAngle);
}

bool SMS_STS::writeMultiCycle(u8 ID) {
	u8 msg[4] {0, 0, 0, 0};
	return genWrite(ID, SMS_STS_MIN_ANGLE_LIMIT_L, msg, 4);
}

bool SMS_STS::writeTorqueLimit(u8 ID, u16 limit_permil) {
	assert(limit_permil <= 1000 && "Torque limit should <= 1000.");
	return writeWord(ID, SMS_STS_MAX_TORQUE, limit_permil);
}

bool SMS_STS::writePositionPID(u8 ID, u8 Pgain, u8 Igain, u8 Dgain) {
	u8 msg[3] {Pgain, Dgain, Igain};
	return genWrite(ID, SMS_STS_POS_P_GAIN, msg, 3);
}

bool SMS_STS::writePositionPgain(u8 ID, u8 Pgain) {
	return writeByte(ID, SMS_STS_POS_P_GAIN, Pgain);
}

bool SMS_STS::writePositionIgain(u8 ID, u8 Igain) {
	return writeByte(ID, SMS_STS_POS_I_GAIN, Igain);
}

bool SMS_STS::writePositionDgain(u8 ID, u8 Dgain) {
	return writeByte(ID, SMS_STS_POS_D_GAIN, Dgain);
}

bool SMS_STS::writePositionCorrection(u8 ID, u16 correction, bool posDirection) {
	assert( correction <= 2047 && "Correction value should be 0~2047");
	if (posDirection)
		correction |= ( 1 << 11);
	return writeWord(ID, SMS_STS_OFS_L, posDirection);
}

bool SMS_STS::writePosition(u8 ID, s16 position) {
	if(position < 0){
		position = -position;
		position |= (1<<15);
	}
	return writeWord(ID, SMS_STS_GOAL_POSITION_L, static_cast<u16>(position));
}

bool SMS_STS::writeAcceleration(u8 ID, u8 acc) {
	return writeByte(ID, SMS_STS_ACC, acc);
}



bool SMS_STS::wheelMode(u8 ID)
{
	return writeByte(ID, SMS_STS_MODE, 1);		
}

bool SMS_STS::writeSpeed(u8 ID, s16 speed, u8 acceleration)
{
	if(speed<0){
		speed = -speed;
		speed |= (1<<15);
	}
	u8 bBuf[2];
	bBuf[0] = acceleration;
	genWrite(ID, SMS_STS_ACC, bBuf, 1);
	Host2SCS(bBuf+0, bBuf+1, speed);
	
	return genWrite(ID, SMS_STS_GOAL_SPEED_L, bBuf, 2);
}

bool SMS_STS::enableTorque(u8 ID)
{
	return writeByte(ID, SMS_STS_TORQUE_ENABLE, 0x01);
}

bool SMS_STS::disableTorque(u8 ID)
{
	return writeByte(ID, SMS_STS_TORQUE_ENABLE, 0x00);
}

bool SMS_STS::unlockEEPROM(u8 ID)
{
	return writeByte(ID, SMS_STS_LOCK, 0);
}

bool SMS_STS::lockEEPROM(u8 ID)
{
	return writeByte(ID, SMS_STS_LOCK, 1);
}

bool SMS_STS::offsetCalibration(u8 ID)
{
	return writeByte(ID, SMS_STS_TORQUE_ENABLE, 128);
}

int SMS_STS::feedback(int ID)
{
	int nLen = read(ID, SMS_STS_PRESENT_POSITION_L, Mem, sizeof(Mem));
	if(nLen!=sizeof(Mem)){
		m_status = true;
		return -1;
	}
	m_status = false;
	return nLen;
}

int SMS_STS::readPosition(int ID)
{
	int Pos = -1;
	if(ID==-1){
		Pos = Mem[SMS_STS_PRESENT_POSITION_H-SMS_STS_PRESENT_POSITION_L];
		Pos <<= 8;
		Pos |= Mem[SMS_STS_PRESENT_POSITION_L-SMS_STS_PRESENT_POSITION_L];
	}else{
		m_status = false;
		Pos = readWord(ID, SMS_STS_PRESENT_POSITION_L);
		if(Pos==-1){
			m_status = true;
		}
	}
	if(!m_status && (Pos&(1<<15))){
		Pos = -(Pos&~(1<<15));
	}
	
	return Pos;
}

int SMS_STS::readSpeed(int ID)
{
	int speed = -1;
	if(ID==-1){
		speed = Mem[SMS_STS_PRESENT_SPEED_H-SMS_STS_PRESENT_POSITION_L];
		speed <<= 8;
		speed |= Mem[SMS_STS_PRESENT_SPEED_L-SMS_STS_PRESENT_POSITION_L];
	}else{
		m_status = false;
		speed = readWord(ID, SMS_STS_PRESENT_SPEED_L);
		if(speed==-1){
			m_status = true;
			return -1;
		}
	}
	if(!m_status && (speed&(1<<15))){
		speed = -(speed&~(1<<15));
	}	
	return speed;
}

int SMS_STS::readLoad(int ID)
{
	int load = -1;
	if(ID==-1){
		load = Mem[SMS_STS_PRESENT_LOAD_H-SMS_STS_PRESENT_POSITION_L];
		load <<= 8;
		load |= Mem[SMS_STS_PRESENT_LOAD_L-SMS_STS_PRESENT_POSITION_L];
	}else{
		m_status = false;
		load = readWord(ID, SMS_STS_PRESENT_LOAD_L);
		if(load==-1){
			m_status = true;
		}
	}
	if(!m_status && (load&(1<<10))){
		load = -(load&~(1<<10));
	}
	return load;
}

int SMS_STS::readVoltage(int ID)
{	
	int voltage = -1;
	if(ID==-1){
		voltage = Mem[SMS_STS_PRESENT_VOLTAGE-SMS_STS_PRESENT_POSITION_L];	
	}else{
		m_status = false;
		voltage = readByte(ID, SMS_STS_PRESENT_VOLTAGE);
		if(voltage==-1){
			m_status = true;
		}
	}
	return voltage;
}

int SMS_STS::readTemp(int ID)
{	
	int temper = -1;
	if(ID==-1){
		temper = Mem[SMS_STS_PRESENT_TEMPERATURE-SMS_STS_PRESENT_POSITION_L];	
	}else{
		m_status = false;
		temper = readByte(ID, SMS_STS_PRESENT_TEMPERATURE);
		if(temper==-1){
			m_status = true;
		}
	}
	return temper;
}

bool SMS_STS::isMoving(int ID)
{
	int move = -1;
	if(ID==-1){
		move = Mem[SMS_STS_MOVING-SMS_STS_PRESENT_POSITION_L];	
	}else{
		m_status = false;
		move = readByte(ID, SMS_STS_MOVING);
		if(move==-1){
			m_status = true;
		}
	}
	return move == 1;
}

int SMS_STS::readMode(int ID)
{
	int mode = -1;
	if(ID==-1){
		mode = Mem[SMS_STS_MODE-SMS_STS_PRESENT_POSITION_L];	
	}else{
		m_status = false;
		mode = readByte(ID, SMS_STS_MODE);
		if(mode==-1){
			m_status = true;
		}
	}
	return mode;
}

int SMS_STS::readCurrent(int ID)
{
	int Current = -1;
	if(ID==-1){
		Current = Mem[SMS_STS_PRESENT_CURRENT_H-SMS_STS_PRESENT_POSITION_L];
		Current <<= 8;
		Current |= Mem[SMS_STS_PRESENT_CURRENT_L-SMS_STS_PRESENT_POSITION_L];
	}else{
		m_status = false;
		Current = readWord(ID, SMS_STS_PRESENT_CURRENT_L);
		if(Current==-1){
			m_status = true;
			return -1;
		}
	}
	if(!m_status && (Current&(1<<15))){
		Current = -(Current&~(1<<15));
	}	
	return Current;
}

