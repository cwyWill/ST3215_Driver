/*
 * SCSerial.h
 * hardware interface layer for waveshare serial bus servo
 * date: 2023.6.28
 */


#include "SCSerial.h"
#include "timer.h"
#include <iostream>

#include <chrono>
#include <boost/asio/steady_timer.hpp>

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
	// std::size_t bytes_read { 0 };
	// bool time_out { false };
	
	// boost::asio::steady_timer timer { m_io };
	
	// timer.expires_after(std::chrono::microseconds(m_IOTimeOut_ms));
	// timer.async_wait([&](const boost::system::error_code& ec){
	// 	if (!ec) {
	// 		time_out = true;
	// 		m_serial.cancel();
	// 	}
	// });

	// // start async read
	// boost::asio::async_read(m_serial, boost::asio::buffer(nDat, nLen), [&](const boost::system::error_code& ec, std::size_t n) {
	// 	if ( !ec ){
	// 		bytes_read = n;
	// 	}
	// });

	// m_io.run();
	// m_io.restart();

	// return static_cast<int>(bytes_read);

	boost::system::error_code ec;
	std::size_t bytes_read {
		boost::asio::read(
			m_serial,
			boost::asio::buffer(nDat, nLen),
			ec
		)
	};
	
	if (ec) {
		return 0;
	}
	return static_cast<int>(bytes_read);
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