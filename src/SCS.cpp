/*
 * SCS.cpp
 * communication layer for serial bus servo
 * date: 2023.6.28
 */

#include <stddef.h>
#include "SCS.h"
#include <iostream>


std::ostream& operator<<(std::ostream& out, ReadStatus status) {
	switch (status)
	{
	case ReadStatus::Okay:
		out << "okay"; break;
	case ReadStatus::Timeout:
		out << "timeout"; break;
	case ReadStatus::ProtocolError:
		out << "protocol"; break;
	case ReadStatus::ReadSessionError:
		out << "session"; break;
	case ReadStatus::HeaderError:
		out << "header"; break;
	case ReadStatus::ID_CheckError:
		out << "ID"; break;
	case ReadStatus::ChecksumError:
		out << "checksum"; break;
	case ReadStatus::LengthError:
		out << "return length"; break;
	case ReadStatus::Abort:
		out << "abort"; break;
	case ReadStatus::Other:
		out << "unknown"; break;
	default:
		out << "what the heck"; break;
	}
	return out;
}

SCS::SCS() :
	SCS{ false, 1}
{ }

SCS::SCS(bool isBigEndian) :
	SCS{ isBigEndian, 1}
{ }

SCS::SCS(bool isBigEndian, u8 level) :
	m_level { level }, 
	m_isBigEndian { isBigEndian }
{ }

// one 16-digit number split into two 8-digit numbers
// dataL is low, dataH is high
void SCS::Host2SCS(u8 *dataL, u8* dataH, u16 data)
{
	if( m_isBigEndian){
		*dataL = (data>>8);
		*dataH = (data&0xff);
	}else{
		*dataH = (data>>8);
		*dataL = (data&0xff);
	}
}

// combination of two 8-digit numbers into one 16-digit number
// dataL is low, dataH is high
u16 SCS::SCS2Host(u8 dataL, u8 dataH)
{
	u16 data;
	if(m_isBigEndian){
		data = dataL;
		data <<= 8;
		data |= dataH;
	}else{
		data = dataH;
		data <<= 8;
		data |= dataL;
	}
	return data;
}

void SCS::writeBuf(int ID, u8 memAddr, u8 *nDat, u8 nLen, u8 instr)
{
	if ( ID > 254 ){
		return;
	}
	u8 msgLen = 2;
	u8 bBuf[6];
	u8 CheckSum = 0;
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = static_cast<u8>(ID);
	bBuf[4] = instr;
	if(nDat){
		msgLen += nLen + 1;
		bBuf[3] = msgLen;
		bBuf[5] = memAddr;
		writeSCS(bBuf, 6);
		
	}else{
		bBuf[3] = msgLen;
		writeSCS(bBuf, 5);
	}
	CheckSum = ID + msgLen + instr + memAddr;
	u8 i = 0;
	if(nDat){
		for(i=0;  i<nLen ; i++){
			CheckSum += nDat[i];
		}
		writeSCS(nDat, nLen);
	}
	writeSCS(~CheckSum);
}

// general write command.
// the ID of the servo, the memory address in memory table, the data to write, the length of data
ReadResult<bool> SCS::genWrite(int ID, u8 memAddr, u8 *nDat, u8 nLen)
{
	rFlushSCS();
	writeBuf(ID, memAddr, nDat, nLen, static_cast<u8>(Instruction::write));
	wFlushSCS();
	return ack(ID);
}

// write asynchronously.
// the ID of the servo，the memory address in memory table，the data to write，the length of data
ReadResult<bool> SCS::regWrite(int ID, u8 memAddr, u8 *nDat, u8 nLen)
{
	rFlushSCS();
	writeBuf(ID, memAddr, nDat, nLen, static_cast<u8>(Instruction::regWrite));
	wFlushSCS();
	return ack(ID);
}

// the trigger command for regWrite()
// call this function to start the regWrite() command
// ID: the ID of the servo
ReadResult<bool> SCS::regWriteAction(int ID)
{
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, static_cast<u8>(Instruction::regAction));
	wFlushSCS();
	return ack(ID);
}

// write synchronously.
// the list of servo IDs, the length(number) of the ID list, the memory address in memory table,
// the data to write, the length of data.
void SCS::syncWrite(int ID[], u8 IDN, u8 memAddr, u8 *nDat, u8 nLen)
{
	rFlushSCS();
	u8 mesLen = ((nLen+1)*IDN+4);
	u8 Sum = 0;
	u8 bBuf[7];
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = 0xfe;
	bBuf[3] = mesLen;
	bBuf[4] = static_cast<u8>(Instruction::syncWrite);
	bBuf[5] = memAddr;
	bBuf[6] = nLen;
	writeSCS(bBuf, 7);

	Sum = 0xfe + mesLen + static_cast<u8>(Instruction::syncWrite) + memAddr + nLen;
	u8 i, j;
	for(i=0; i<IDN; i++){
		writeSCS(ID[i]);
		writeSCS(nDat+i*nLen, nLen);
		Sum += ID[i];
		for(j=0; j<nLen; j++){
			Sum += nDat[i*nLen+j];
		}
	}
	writeSCS(~Sum);
	wFlushSCS();
}

ReadResult<bool> SCS::writeByte(int ID, u8 memAddr, u8 bDat)
{
	rFlushSCS();
	writeBuf(ID, memAddr, &bDat, 1, static_cast<u8>(Instruction::write));
	wFlushSCS();
	return ack(ID);
}

ReadResult<bool> SCS::writeWord(int ID, u8 memAddr, u16 wDat)
{
	u8 bBuf[2];
	Host2SCS(bBuf+0, bBuf+1, wDat);
	rFlushSCS();
	writeBuf(ID, memAddr, bBuf, 2, static_cast<u8>(Instruction::write));
	wFlushSCS();
	return ack(ID);
}

// read command
// the ID of servo, the memory address in memory table, the return data, the length of data
ReadResult<int> SCS::read(int ID, u8 memAddr, u8 nData[], u8 nLen)
{
	rFlushSCS();
	writeBuf(ID, memAddr, &nLen, 1, static_cast<u8>(Instruction::read));
	wFlushSCS();
	if( !checkHead() ) {
		return {0, ReadStatus::HeaderError};
	}
	u8 bBuf[4];
	if ( readSCS(bBuf, 3) != 3 ) {
		return {0, ReadStatus::ReadSessionError};
	}
	u8 returnID = bBuf[0];
	if ( returnID != ID ) {
		return {0, ReadStatus::ID_CheckError};
	}
	u8 dataLength = bBuf[1];
	if ( dataLength != (nLen+2) ) {
		return {0, ReadStatus::LengthError};
	}
	u8 workingError = bBuf[2];
	int size = readSCS(nData, nLen);
	if( size != nLen ) {
		return {0, ReadStatus::LengthError};
	}
	if ( readSCS(bBuf+3, 1) != 1 ) {
		return {0, ReadStatus::ReadSessionError};
	}
	u8 calSum = bBuf[0] + bBuf[1] + bBuf[2];
	for(int i=0; i<size; i++) {
		calSum += nData[i];
	}
	calSum = ~calSum;

	if(calSum != bBuf[3]) {
		return {0, ReadStatus::ChecksumError};
	}
	return {size, ReadStatus::Okay, workingError};
}

// read 1 byte from servo, return -1 when timeout
ReadResult<int> SCS::readByte(int ID, u8 memAddr)
{
	u8 bDat;
	ReadResult<int> result = read(ID, memAddr, &bDat, 1);
	if (result.status != ReadStatus::Okay) {
		return {0, result.status};
	}
	return {static_cast<int>(bDat), result.status, result.workingError};
}

// read 2 byte from servo, return -1 when timeout
ReadResult<int> SCS::readWord(int ID, u8 memAddr)
{	
	u8 nDat[2];
	u16 wDat;
	ReadResult<int> result = read(ID, memAddr, nDat, 2);
	if (result.status != ReadStatus::Okay)
		return {0, result.status};
	wDat = SCS2Host(nDat[0], nDat[1]);
	return {static_cast<int>(wDat), result.status, result.workingError};
}

// Ping command, return the ID of servo, return -1 when timeout.
ReadResult<bool> SCS::ping(int ID)
{
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, static_cast<u8>(Instruction::ping));
	wFlushSCS();
	if ( !checkHead() ){
		return {false, ReadStatus::HeaderError};
	}
	u8 bBuf[4];
	if ( readSCS(bBuf, 4) != 4 ) {
		return {false, ReadStatus::ReadSessionError};
	}
	if ( bBuf[0] != ID && ID != 0xfe ) {
		return {false, ReadStatus::ID_CheckError};
	}
	if (bBuf[1]!=2){
		return {false, ReadStatus::LengthError};
	}
	u8 workingError { bBuf[2] };
	u8 calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
	if ( calSum != bBuf[3] ) {
		return {false, ReadStatus::ChecksumError};
	}
	return { (workingError == 0), ReadStatus::Okay, workingError};
}

bool SCS::checkHead()
{
	u8 prev { 0 };
	u8 curr { 0 };

	for (int i = 0; i < 100; ++i)
    {
        if (readSCS(&curr, 1) != 1)
            return false;

        if (prev == 0xFF && curr == 0xFF)
            return true;

        prev = curr;
    }
    return false;

	// u8 bDat;
	// u8 bBuf[2] {0, 0};
	// int cnt { 0 };
	// while( true ){
	// 	if(!readSCS(&bDat, 1)){
	// 		return false;
	// 	}
	// 	bBuf[1] = bBuf[0];
	// 	bBuf[0] = bDat;
	// 	if ( bBuf[0]==0xff && bBuf[1]==0xff ){
	// 		break;
	// 	}
	// 	++cnt;
	// 	if( cnt > 10 ){
	// 		return false;
	// 	}
	// }
	// return true;
}

ReadResult<bool> SCS::ack(int ID)
{
	if ( ID == 0xFE ) {
		return {false, ReadStatus::Broadcast, 0 };
	}
	if(!checkHead()){
		return { false, ReadStatus::HeaderError, 0 };
	}
	u8 bBuf[4];
	if (readSCS(bBuf, 4)!=4){
		return { false, ReadStatus::ReadSessionError, 0 };
	}
	if (bBuf[0] != ID){
		return { false, ReadStatus::ID_CheckError, 0 };
	}
	if (bBuf[1] != 2){
		return { false, ReadStatus::LengthError, 0 };
	}
	u8 calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
	if (calSum != bBuf[3]){
		return { false, ReadStatus::ChecksumError, 0 };
	}
	u8 workingError { bBuf[2] };
	return { true, ReadStatus::Okay, workingError };
}

int	SCS::syncReadPacketTx(u8 ID[], u8 IDN, u8 memAddr, u8 nLen)
{
	syncReadRxPacketLen = nLen;
	u8 checkSum = (4+0xfe)+IDN+memAddr+nLen+ static_cast<u8>(Instruction::syncRead);
	u8 i;
	writeSCS(0xff);
	writeSCS(0xff);
	writeSCS(0xfe);
	writeSCS(IDN+4);
	writeSCS(static_cast<u8>(Instruction::syncRead));
	writeSCS(memAddr);
	writeSCS(nLen);
	for(i=0; i<IDN; i++){
		writeSCS(ID[i]);
		checkSum += ID[i];
	}
	checkSum = ~checkSum;
	writeSCS(checkSum);
	return nLen;
}

int SCS::syncReadPacketRx(u8 ID, u8 *nDat)
{
	syncReadRxPacket = nDat;
	syncReadRxPacketIndex = 0;
	u8 bBuf[4];
	if(!checkHead()){
		return 0;
	}
	if(readSCS(bBuf, 3)!=3){
		return 0;
	}
	if(bBuf[0]!=ID){
		return 0;
	}
	if(bBuf[1]!=(syncReadRxPacketLen+2)){
		return 0;
	}
	[[maybe_unused]] u8 workingError { bBuf[2] };
	if(readSCS(nDat, syncReadRxPacketLen)!=syncReadRxPacketLen){
		return 0;
	}
	return syncReadRxPacketLen;
}

int SCS::syncReadRxPacketToByte()
{
	if(syncReadRxPacketIndex>=syncReadRxPacketLen){
		return -1;
	}
	return syncReadRxPacket[syncReadRxPacketIndex++];
}

int SCS::syncReadRxPacketToWord(u8 negBit)
{
	if((syncReadRxPacketIndex+1)>=syncReadRxPacketLen){
		return -1;
	}
	int Word = SCS2Host(syncReadRxPacket[syncReadRxPacketIndex], syncReadRxPacket[syncReadRxPacketIndex+1]);
	syncReadRxPacketIndex += 2;
	if(negBit){
		if(Word&(1<<negBit)){
			Word = -(Word & ~(1<<negBit));
		}
	}
	return Word;
}
