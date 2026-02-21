/*
 * SCSerial.h
 * hardware interface layer for waveshare serial bus servo
 * date: 2023.6.28 
 */

#ifndef _SCSERIAL_H
#define _SCSERIAL_H

#include "SCS.h"
#include <boost/asio.hpp>
#include <string>

class SCSerial : public SCS
{
// member variables
public:
	boost::asio::io_context m_io;
	std::string m_port {"/dev/ttyACM0"};
	BaudRate m_baudrate {BaudRate::r_1M};
	boost::asio::serial_port m_serial;
	int m_IOTimeOut_ms {100};  //I/O timeout

// getters and setters
public:
	void setTimeout_ms(int timeout_ms) { m_IOTimeOut_ms = timeout_ms; }

// constructors
public:
	SCSerial(std::string port, BaudRate rate);
	SCSerial(std::string port, BaudRate rate, bool isBigEndian);
	SCSerial(std::string port, BaudRate rate, bool isBigEndian, u8 level);
	virtual ~SCSerial() = default;

protected:
	// write to serial
	virtual void writeSCS(unsigned char *nDat, int nLen);
	virtual void writeSCS(unsigned char bDat);
	// read from serial
	virtual int readSCS(unsigned char *nDat, int nLen); 	// input nLen byte

	// flushing the read/write buffer
	virtual void rFlushSCS();
	virtual void wFlushSCS();
};

#endif