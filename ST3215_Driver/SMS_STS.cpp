/*
 * SMS_STS.cpp
 * application layer for waveshare ST servos
 * date: 2023.6.17 
 */

#include "SMS_STS.h"
#include <iostream>

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

void SMS_STS::syncWritePosEx(u8 ID[], u8 IDN, s16 position[], u16 speed[], u8 acceleration[])
{
    u8 offbuf[7*IDN];
    for(u8 i = 0; i<IDN; i++){
		if(position[i]<0){
			position[i] = -position[i];
			position[i] |= (1<<15);
		}
		u16 V;
		if(speed){
			V = speed[i];
		}else{
			V = 0;
		}
		if(acceleration){
			offbuf[i*7] = acceleration[i];
		}else{
			offbuf[i*7] = 0;
		}
        Host2SCS(offbuf+i*7+1, offbuf+i*7+2, position[i]);
        Host2SCS(offbuf+i*7+3, offbuf+i*7+4, 0);
        Host2SCS(offbuf+i*7+5, offbuf+i*7+6, V);
    }
    syncWrite(ID, IDN, SMS_STS_ACC, offbuf, 7);
}

int SMS_STS::wheelMode(u8 ID)
{
	return writeByte(ID, SMS_STS_MODE, 1);		
}

int SMS_STS::writeSpeed(u8 ID, s16 speed, u8 acceleration)
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

int SMS_STS::enableTorque(u8 ID)
{
	return writeByte(ID, SMS_STS_TORQUE_ENABLE, 0x01);
}

int SMS_STS::disableTorque(u8 ID)
{
	return writeByte(ID, SMS_STS_TORQUE_ENABLE, 0x00);
}

int SMS_STS::unlockEEPROM(u8 ID)
{
	return writeByte(ID, SMS_STS_LOCK, 0);
}

int SMS_STS::lockEEPROM(u8 ID)
{
	return writeByte(ID, SMS_STS_LOCK, 1);
}

int SMS_STS::offsetCalibration(u8 ID)
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
	int Speed = -1;
	if(ID==-1){
		Speed = Mem[SMS_STS_PRESENT_SPEED_H-SMS_STS_PRESENT_POSITION_L];
		Speed <<= 8;
		Speed |= Mem[SMS_STS_PRESENT_SPEED_L-SMS_STS_PRESENT_POSITION_L];
	}else{
		m_status = false;
		Speed = readWord(ID, SMS_STS_PRESENT_SPEED_L);
		if(Speed==-1){
			m_status = true;
			return -1;
		}
	}
	if(!m_status && (Speed&(1<<15))){
		Speed = -(Speed&~(1<<15));
	}	
	return Speed;
}

int SMS_STS::readLoad(int ID)
{
	int Load = -1;
	if(ID==-1){
		Load = Mem[SMS_STS_PRESENT_LOAD_H-SMS_STS_PRESENT_POSITION_L];
		Load <<= 8;
		Load |= Mem[SMS_STS_PRESENT_LOAD_L-SMS_STS_PRESENT_POSITION_L];
	}else{
		m_status = false;
		Load = readWord(ID, SMS_STS_PRESENT_LOAD_L);
		if(Load==-1){
			m_status = true;
		}
	}
	if(!m_status && (Load&(1<<10))){
		Load = -(Load&~(1<<10));
	}
	return Load;
}

int SMS_STS::readVoltage(int ID)
{	
	int Voltage = -1;
	if(ID==-1){
		Voltage = Mem[SMS_STS_PRESENT_VOLTAGE-SMS_STS_PRESENT_POSITION_L];	
	}else{
		m_status = false;
		Voltage = readByte(ID, SMS_STS_PRESENT_VOLTAGE);
		if(Voltage==-1){
			m_status = true;
		}
	}
	return Voltage;
}

int SMS_STS::readTemp(int ID)
{	
	int Temper = -1;
	if(ID==-1){
		Temper = Mem[SMS_STS_PRESENT_TEMPERATURE-SMS_STS_PRESENT_POSITION_L];	
	}else{
		m_status = false;
		Temper = readByte(ID, SMS_STS_PRESENT_TEMPERATURE);
		if(Temper==-1){
			m_status = true;
		}
	}
	return Temper;
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

