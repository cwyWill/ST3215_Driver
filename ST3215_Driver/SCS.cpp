/*
 * SCS.cpp
 * communication layer for serial bus servo
 * date: 2023.6.28
 */

#include <stddef.h>
#include "SCS.h"
#include <iostream>

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

void SCS::writeBuf(u8 ID, u8 memAddr, u8 *nDat, u8 nLen, u8 instr)
{
	u8 msgLen = 2;
	u8 bBuf[6];
	u8 CheckSum = 0;
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = ID;
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
bool SCS::genWrite(u8 ID, u8 memAddr, u8 *nDat, u8 nLen)
{
	rFlushSCS();
	writeBuf(ID, memAddr, nDat, nLen, static_cast<u8>(Instruction::write));
	wFlushSCS();
	return ack(ID);
}

// write asynchronously.
// the ID of the servo，the memory address in memory table，the data to write，the length of data
bool SCS::regWrite(u8 ID, u8 memAddr, u8 *nDat, u8 nLen)
{
	rFlushSCS();
	writeBuf(ID, memAddr, nDat, nLen, static_cast<u8>(Instruction::regWrite));
	wFlushSCS();
	return ack(ID);
}

// the trigger command for regWrite()
// call this function to start the regWrite() command
// ID: the ID of the servo
bool SCS::RegWriteAction(u8 ID)
{
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, static_cast<u8>(Instruction::regAction));
	wFlushSCS();
	return ack(ID);
}

// write synchronously.
// the list of servo IDs, the length(number) of the ID list, the memory address in memory table,
// the data to write, the length of data.
void SCS::syncWrite(u8 ID[], u8 IDN, u8 memAddr, u8 *nDat, u8 nLen)
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

bool SCS::writeByte(u8 ID, u8 memAddr, u8 bDat)
{
	rFlushSCS();
	writeBuf(ID, memAddr, &bDat, 1, static_cast<u8>(Instruction::write));
	wFlushSCS();
	return ack(ID);
}

bool SCS::writeWord(u8 ID, u8 memAddr, u16 wDat)
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
int SCS::read(u8 ID, u8 memAddr, u8 nData[], u8 nLen)
{
	rFlushSCS();
	writeBuf(ID, memAddr, &nLen, 1, static_cast<u8>(Instruction::read));
	wFlushSCS();
	if(!checkHead()){
		return 0;
	}
	u8 bBuf[4];
	m_error = 0;
	if ( readSCS(bBuf, 3) != 3 ) {
		return 0;
	}
	int size = readSCS(nData, nLen);
	if( size != nLen ) {
		return 0;
	}
	if ( readSCS(bBuf+3, 1) != 1 ) {
		return 0;
	}
	u8 calSum = bBuf[0] + bBuf[1] + bBuf[2];
	for(int i=0; i<size; i++) {
		calSum += nData[i];
	}
	calSum = ~calSum;

	if(calSum != bBuf[3]) {
		return 0;
	}
	m_error = bBuf[2];
	return size;
}

// read 1 byte from servo, return -1 when timeout
int SCS::readByte(u8 ID, u8 memAddr)
{
	u8 bDat;
	int size = read(ID, memAddr, &bDat, 1);
	if (size != 1) {
		return -1;
	}
	else {
		return bDat;
	}
}

// read 2 byte from servo, return -1 when timeout
int SCS::readWord(u8 ID, u8 memAddr)
{	
	u8 nDat[2];
	int Size;
	u16 wDat;
	Size = read(ID, memAddr, nDat, 2);
	if(Size!=2)
		return -1;
	wDat = SCS2Host(nDat[0], nDat[1]);
	return wDat;
}

// Ping command, return the ID of servo, return -1 when timeout.
int	SCS::ping(u8 ID)
{
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, static_cast<u8>(Instruction::ping));
	wFlushSCS();
	m_error = 0;
	if(!checkHead()){
		return -1;
	}
	u8 bBuf[4];
	if(readSCS(bBuf, 4)!=4){
		return -1;
	}
	if(bBuf[0]!=ID && ID!=0xfe){
		return -1;
	}
	if(bBuf[1]!=2){
		return -1;
	}
	u8 calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
	if(calSum!=bBuf[3]){
		return -1;			
	}
	m_error = bBuf[2];
	return bBuf[0];
}

bool SCS::checkHead()
{
	u8 bDat;
	u8 bBuf[2] {0, 0};
	u8 cnt { 0 };
	while( true ){
		if(!readSCS(&bDat, 1)){
			return false;
		}
		bBuf[1] = bBuf[0];
		bBuf[0] = bDat;
		if ( bBuf[0]==0xff && bBuf[1]==0xff ){
			break;
		}
		cnt++;
		if( cnt > 10 ){
			return false;
		}
	}
	return true;
}

bool SCS::ack(u8 ID)
{
	m_error = 0;
	if(ID!=0xfe && m_level){
		if(!checkHead()){
			return false;
		}
		u8 bBuf[4];
		if (readSCS(bBuf, 4)!=4){
			return false ;
		}
		if (bBuf[0] != ID){
			return false;
		}
		if (bBuf[1] != 2){
			return false;
		}
		u8 calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
		if (calSum != bBuf[3]){
			return 0;			
		}
		m_error = bBuf[2];
	}
	return true;
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
	m_error = bBuf[2];
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
