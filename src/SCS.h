/*
 * SCS.h
 * communication layer for waveshare serial bus servo
 * date: 2019.12.18 
 */

#ifndef _SCS_H
#define _SCS_H

#include "INST.h"

class SCS {
// member variables
public:
	u8 m_level {1}; // the level of the servo return
	bool m_isBigEndian { false }; // processor endian structure
	u8 m_error { 0 }; // the status of servo
	u8 syncReadRxPacketIndex;
	u8 syncReadRxPacketLen;
	u8 *syncReadRxPacket;

// constructor
public:
	SCS();
	SCS(bool isBigEndian);
	SCS(bool isBigEndian, u8 level);

public:
	bool genWrite(u8 ID, u8 memAddr, u8 *nDat, u8 nLen); // general write
	bool regWrite(u8 ID, u8 memAddr, u8 *nDat, u8 nLen); // write asynchronously
	bool RegWriteAction(u8 ID = 0xfe); // trigger command for regWrite()
	void syncWrite(u8 ID[], u8 IDN, u8 memAddr, u8 *nDat, u8 nLen); // write synchronously
	bool writeByte(u8 ID, u8 memAddr, u8 bDat); // write 1 byte
	bool writeWord(u8 ID, u8 memAddr, u16 wDat); // write 2 byte
	int read(u8 ID, u8 memAddr, u8 *nData, u8 nLen); // read command
	int readByte(u8 ID, u8 memAddr); // read 1 byte
	int readWord(u8 ID, u8 memAddr); // read 2 byte
	bool ping(u8 ID); // Ping command
	int syncReadPacketTx(u8 ID[], u8 IDN, u8 memAddr, u8 nLen); // read synchronously command send
	int syncReadPacketRx(u8 ID, u8 *nDat); // read synchronously command receive, return the number of byte when succeed, return 0 when failed
	int syncReadRxPacketToByte(); // decode one byte
	int syncReadRxPacketToWord(u8 negBit=0); // decode 2 byte, negBit is the direction, 0 as none.

// serial operation
protected:
	virtual void writeSCS(unsigned char *nDat, int nLen) = 0;
	virtual int readSCS(unsigned char *nDat, int nLen) = 0;
	virtual void writeSCS(unsigned char bDat) = 0;
	virtual void rFlushSCS() = 0;
	virtual void wFlushSCS() = 0;
protected:
	void writeBuf(u8 ID, u8 memAddr, u8 *nDat, u8 nLen, u8 instr);
	void Host2SCS(u8 *dataL, u8* dataH, u16 data); // one 16-digit number split into two 8-digit numbers
	u16	SCS2Host(u8 dataL, u8 dataH); // combination of two 8-digit numbers into one 16-digit number
	bool ack(u8 ID); // return response
	bool checkHead(); // Frame header detection
};
#endif
