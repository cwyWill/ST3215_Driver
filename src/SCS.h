/*
 * SCS.h
 * communication layer for waveshare serial bus servo
 * date: 2019.12.18 
 */

#ifndef _SCS_H
#define _SCS_H

#include "INST.h"
#include <iostream>

enum class ReadStatus {
	Okay,
	Broadcast,
	Timeout,
	ProtocolError,
	ReadSessionError,
	HeaderError,
	ID_CheckError,
	ChecksumError,
	LengthError,
	Abort,
	Other,
};

constexpr bool operator!(ReadStatus status) noexcept
{
    return status != ReadStatus::Okay;
}


template <typename T>
struct ReadResult {
	T value {};
	ReadStatus status { ReadStatus::Okay };
	u8 workingError { 0 }; // the error code returned by servo, only valid when status is Okay
};

template <>
struct ReadResult<bool> {
	bool okay { true };
	ReadStatus status { ReadStatus::Okay };
	u8 workingError { 0 };
};

std::ostream& operator<<(std::ostream& out, ReadStatus status);


class SCS {
// member variables
protected:
	u8 m_level {1}; // the level of the servo return
	bool m_isBigEndian { false }; // processor endian structure
	u8 syncReadRxPacketIndex;
	u8 syncReadRxPacketLen;
	u8 *syncReadRxPacket;

	// ReadStatus m_ackg { ReadStatus::Okay };

// constructor
public:
	SCS();
	SCS(bool isBigEndian);
	SCS(bool isBigEndian, u8 level);

public:
	ReadResult<bool> genWrite(int ID, u8 memAddr, u8 *nDat, u8 nLen); // general write
	ReadResult<bool> regWrite(int ID, u8 memAddr, u8 *nDat, u8 nLen); // write asynchronously
	ReadResult<bool> regWriteAction(int ID = 0xfe); // trigger command for regWrite()
	void syncWrite(int ID[], u8 IDN, u8 memAddr, u8 *nDat, u8 nLen); // write synchronously
	ReadResult<bool> writeByte(int ID, u8 memAddr, u8 bDat); // write 1 byte
	ReadResult<bool> writeWord(int ID, u8 memAddr, u16 wDat); // write 2 byte

	ReadResult<int> read(int ID, u8 memAddr, u8 *nData, u8 nLen); // read command
	ReadResult<int> readByte(int ID, u8 memAddr); // read 1 byte
	ReadResult<int> readWord(int ID, u8 memAddr); // read 2 byte
	ReadResult<bool> ping(int ID); // Ping command

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
	void writeBuf(int ID, u8 memAddr, u8 *nDat, u8 nLen, u8 instr);
	void Host2SCS(u8 *dataL, u8* dataH, u16 data); // one 16-digit number split into two 8-digit numbers
	u16	SCS2Host(u8 dataL, u8 dataH); // combination of two 8-digit numbers into one 16-digit number
	ReadResult<bool> ack(int ID); // return response
	bool checkHead(); // Frame header detection
};
#endif
