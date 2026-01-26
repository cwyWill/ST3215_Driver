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

int SMS_STS::writePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC)
{
	if(Position<0){
		Position = -Position;
		Position |= (1<<15);
	}
	u8 bBuf[7];
	bBuf[0] = ACC;
	Host2SCS(bBuf+1, bBuf+2, Position);
	Host2SCS(bBuf+3, bBuf+4, 0);
	Host2SCS(bBuf+5, bBuf+6, Speed);
	
	return genWrite(ID, SMS_STS_ACC, bBuf, 7);
}

int SMS_STS::regWritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC)
{
	if(Position<0){
		Position = -Position;
		Position |= (1<<15);
	}
	u8 bBuf[7];
	bBuf[0] = ACC;
	Host2SCS(bBuf+1, bBuf+2, Position);
	Host2SCS(bBuf+3, bBuf+4, 0);
	Host2SCS(bBuf+5, bBuf+6, Speed);
	
	return regWrite(ID, SMS_STS_ACC, bBuf, 7);
}

void SMS_STS::syncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[])
{
    u8 offbuf[7*IDN];
    for(u8 i = 0; i<IDN; i++){
		if(Position[i]<0){
			Position[i] = -Position[i];
			Position[i] |= (1<<15);
		}
		u16 V;
		if(Speed){
			V = Speed[i];
		}else{
			V = 0;
		}
		if(ACC){
			offbuf[i*7] = ACC[i];
		}else{
			offbuf[i*7] = 0;
		}
        Host2SCS(offbuf+i*7+1, offbuf+i*7+2, Position[i]);
        Host2SCS(offbuf+i*7+3, offbuf+i*7+4, 0);
        Host2SCS(offbuf+i*7+5, offbuf+i*7+6, V);
    }
    syncWrite(ID, IDN, SMS_STS_ACC, offbuf, 7);
}

int SMS_STS::wheelMode(u8 ID)
{
	return writeByte(ID, SMS_STS_MODE, 1);		
}

int SMS_STS::writeSpeed(u8 ID, s16 Speed, u8 ACC)
{
	if(Speed<0){
		Speed = -Speed;
		Speed |= (1<<15);
	}
	u8 bBuf[2];
	bBuf[0] = ACC;
	genWrite(ID, SMS_STS_ACC, bBuf, 1);
	Host2SCS(bBuf+0, bBuf+1, Speed);
	
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

int SMS_STS::unLockEprom(u8 ID)
{
	return writeByte(ID, SMS_STS_LOCK, 0);
}

int SMS_STS::LockEprom(u8 ID)
{
	return writeByte(ID, SMS_STS_LOCK, 1);
}

int SMS_STS::CalibrationOfs(u8 ID)
{
	return writeByte(ID, SMS_STS_TORQUE_ENABLE, 128);
}

int SMS_STS::FeedBack(int ID)
{
	int nLen = Read(ID, SMS_STS_PRESENT_POSITION_L, Mem, sizeof(Mem));
	if(nLen!=sizeof(Mem)){
		m_status = true;
		return -1;
	}
	m_status = false;
	return nLen;
}

int SMS_STS::ReadPos(int ID)
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

int SMS_STS::ReadSpeed(int ID)
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

int SMS_STS::ReadLoad(int ID)
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

int SMS_STS::ReadVoltage(int ID)
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

int SMS_STS::ReadTemper(int ID)
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

int SMS_STS::ReadMove(int ID)
{
	int Move = -1;
	if(ID==-1){
		Move = Mem[SMS_STS_MOVING-SMS_STS_PRESENT_POSITION_L];	
	}else{
		m_status = false;
		Move = readByte(ID, SMS_STS_MOVING);
		if(Move==-1){
			m_status = true;
		}
	}
	return Move;
}

int SMS_STS::ReadMode(int ID)
{
	int Mode = -1;
	if(ID==-1){
		Mode = Mem[SMS_STS_MODE-SMS_STS_PRESENT_POSITION_L];	
	}else{
		m_status = false;
		Mode = readByte(ID, SMS_STS_MODE);
		if(Mode==-1){
			m_status = true;
		}
	}
	return Mode;
}

int SMS_STS::ReadCurrent(int ID)
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

