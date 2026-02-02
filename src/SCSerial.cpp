/*
 * SCSerial.h
 * hardware interface layer for waveshare serial bus servo
 * date: 2023.6.28
 */


#include "SCSerial.h"
#include "timer.h"
#include <iostream>

SCSerial::SCSerial(std::string port, BaudRate rate) :
	SCSerial{ port, rate, false }
{ }

SCSerial::SCSerial(std::string port, BaudRate rate, bool isBigEndian) :
	SCSerial { port, rate, isBigEndian, 1 }
{ }

SCSerial::SCSerial(std::string port, BaudRate rate, bool isBigEndian, u8 level) :
	SCS{ isBigEndian, level }, m_io {}, m_port { port }, m_baudrate { rate }, m_serial(m_io, port)
{
    m_serial.set_option(boost::asio::serial_port_base::baud_rate(getIntBaudrate(m_baudrate)));
    m_serial.set_option(boost::asio::serial_port_base::character_size(8));
    m_serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    m_serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    m_serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
}

int SCSerial::readSCS(unsigned char *nDat, int nLen)
{
	int size = 0;
	u8 comData;
	Timer timer;

	while ( size < nLen ) {
		boost::asio::read(m_serial, boost::asio::buffer(&comData, 1));
		if( nDat ){
			nDat[size] = comData;
		}
		size++;
	}
	return size;
}

void SCSerial::writeSCS(unsigned char *nDat, int nLen)
{
	if(nDat==NULL){
		std::cerr << "No message sent.";
		return ;
	}
	// write to serial
    boost::asio::write(m_serial, boost::asio::buffer(nDat, nLen));
}

void SCSerial::writeSCS(unsigned char bDat)
{
	// write to serial
    boost::asio::write(m_serial, boost::asio::buffer(&bDat, 1));
}

// TODO read flush buffer timeout handling
void SCSerial::rFlushSCS()
{
	// while ( true ){
	// 	char trash {};
	// 	boost::asio::read(m_serial, boost::asio::buffer(&trash, 1));
	// 	if (trash == '\0') break;
	// }
}

// TODO write flush buffer
void SCSerial::wFlushSCS()
{
}