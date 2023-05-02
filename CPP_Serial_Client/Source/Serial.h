///////////////////////////////////////////////////////////////////////////////
//!
//! @file		Serial.h
//! 
//! @brief		A cross platform class to handle interaction with serial ports.
//! 
//! @author		Chip Brommer
//! 
//! @date		< 04 / 30 / 2023 > Initial Start Date
//!
/*****************************************************************************/
#pragma once
///////////////////////////////////////////////////////////////////////////////
//
//  Includes:
//          name                        reason included
//          --------------------        ---------------------------------------
#ifdef WIN32
#include <stdint.h>						// Standard integer types
#include <windows.h>
#elif defined __linux__
#include <termios.h>					
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#endif
#include <string>						// Strings
#include "Serial_Info.h"
//
//	Defines:
//          name                        reason defined
//          --------------------        ---------------------------------------
#ifndef     CPP_SERIAL					// Define the cpp logger class. 
#define     CPP_SERIAL
//
///////////////////////////////////////////////////////////////////////////////

namespace Essentials
{
	namespace Communications
	{
		class Serial
		{
		public:
			/// <summary>Default Constructor</summary>
			Serial();

			/// <summary>Constructor that accepts some default parameters.</summary>
			/// <param name="port"> -[in]- Port to connect on.</param>
			/// <param name="baud"> -[in]- Baudrate for the port.</param>
			/// <param name="bytes"> -[in]- BytesSize for the port.</param>
			/// <param name="parity"> -[in]- Parity for the port.</param>
			Serial(const std::string port, const BaudRate baud, const ByteSize bytes, const Parity parity);

			/// <summary>Default Deconstructor</summary>
			~Serial();

			/// <summary>Initializer that accepts some default parameters if the default constructor is used.</summary>
			/// <param name="port"> -[in]- Port to connect on.</param>
			/// <param name="baud"> -[in]- Baudrate for the port.</param>
			/// <param name="bytes"> -[in]- BytesSize for the port.</param>
			/// <param name="parity"> -[in]- Parity for the port.</param>
			/// <returns>0 if successful, -1 if fails. Call GetLastError to find out more.</returns>
			int8_t Configure(const std::string port, const BaudRate baud, const ByteSize bytes, const Parity parity);

			/// <summary>Opens a serial connection.</summary>
			/// <returns>0 if successful, -1 if fails. Call GetLastError to find out more.</returns>
			int8_t Open();

			/// <summary>Check if port is open.</summary>
			/// <returns>True if open, false if closed.</returns>
			bool IsOpen();

			/// <summary></summary>
			/// <returns></returns>
			int8_t WaitReadable();
			int8_t Read();
			int8_t ReadLine();
			int8_t Flush();
			int8_t FlushInput();
			int8_t FlushOutput();
			int8_t Write();
			int8_t WriteBreak();

			/// <summary>Closes a serial connection</summary>
			/// <returns>0 if successful, -1 if fails. Call GetLastError to find out more.</returns>
			int8_t Close();

			/// <summary>Sets the port to connect on</summary>
			/// <param name="port"> -[in]- Port to connect on.</param>
			/// <returns>0 if successful, -1 if fails. Call GetLastError to find out more.</returns>
			int8_t SetSerialPort(const std::string port);

			/// <summary>Sets the baudrate for the serial port</summary>
			/// <param name="baud"> -[in]- Baudrate for the port.</param>
			/// <param name="custom"> -[opt/in]- Specified Baudrate when using BaudRate::BAUDRATE_CUSTOM</param>
			/// <returns>0 if successful, -1 if fails. Call GetLastError to find out more.</returns>
			int8_t SetBaudrate(const BaudRate baud, const uint32_t custom = 0);

			/// <summary>Sets the parity for the serial port</summary>
			/// <param name="parity"> -[in]- Parity for the port.</param>
			/// <returns>0 if successful, -1 if fails. Call GetLastError to find out more.</returns>
			int8_t SetParity(const Parity parity);

			/// <summary>Sets the bytesize for the serial port</summary>
			/// <param name="size"> -[in]- BytesSize for the port.</param>
			/// <returns>0 if successful, -1 if fails. Call GetLastError to find out more.</returns>
			int8_t SetByteSize(const ByteSize size);

			/// <summary>Sets the timeout for the serial port</summary>
			/// <param name="timeoutMS"> -[in]- Timeout for the serial port in MSecs</param>
			/// <returns>0 if successful, -1 if fails. Call GetLastError to find out more.</returns>
			int8_t SetTimeout(const uint32_t timeoutMS);

			/// <summary>Sets the stopbits for the serial port</summary>
			/// <param name="bits"> -[in]- StopBits for the port.</param>
			/// <returns>0 if successful, -1 if fails. Call GetLastError to find out more.</returns>
			int8_t SetStopBits(const StopBits bits);

			/// <summary>Sets the flow controller for the serial port</summary>
			/// <param name="flow"> -[in]- Desired flow control designation for the port.</param>
			/// <returns>0 if successful, -1 if fails. Call GetLastError to find out more.</returns>
			int8_t SetFlowControl(const FlowControl flow);

			int8_t SetDelimiter(const uint8_t delimiter);
			int8_t SetBreak();
			int8_t SetRTS();
			int8_t SetDTR();
			int8_t SetBinary();

			/// <summary>Get the current port in string format</summary>
			/// <returns>The set port</returns>
			std::string GetPort();

			/// <summary>Get the current BaudRate set for the port</summary>
			/// <returns>The current baud rate</returns>
			BaudRate GetBaudrate();

			/// <summary>Get the current Parity set for the port</summary>
			/// <returns>The current parity</returns>
			Parity GetParity();

			/// <summary>Get the current ByteSize set for the port</summary>
			/// <returns>The current byte size</returns>
			ByteSize GetByteSize();

			/// <summary>Get the current timeout set for the port</summary>
			/// <returns>The current timeout, 0 = not set</returns>
			uint32_t GetTimeout();

			/// <summary>Get the current StopBits set for the port</summary>
			/// <returns>The current StopBits</returns>
			StopBits GetStopBits();

			/// <summary>Get the current Flow Controller set for the port</summary>
			/// <returns>The current Flow Control setting</returns>
			FlowControl GetFlowControl();

			/// <summary>Get the current delimiter set for the port</summary>
			/// <returns>The current delimiter, 0x00 = not set</returns>
			uint8_t GetDelimiter();


			int8_t GetCTS();
			int8_t GetDSR();
			int8_t GetRI();
			int8_t GetCD();
			bool GetBinary();

			/// <summary></summary>
			/// <returns></returns>
			int32_t GetInQueueLength();

			/// <summary>Get the last error in string format</summary>
			/// <returns>The last error in a formatted string</returns>
			std::string GetLastError();

		protected:
		private:

			int8_t SetCustomBaudrate();

			std::string		mTitle;				// Title for the class when using CPP_LOGGER
			std::string		mPort;				// Port for the serial connection
			BaudRate		mBaudRate;			// BaudRate for the serial connection
			uint32_t		mCustomBaudRate;	// Custom BaudRate holder. 
			Parity			mParity;			// Parity for the serial connection
			ByteSize		mByteSize;			// Byte Size for the serial connection
			uint32_t		mTimeout;			// Timeout for the serial connection
			StopBits		mStopBits;			// Stop Bits for the serial connection
			FlowControl		mFlowControl;		// Flow Control for the serial connection
			SerialError		mLastError;			// Last error for this utility
			uint8_t			mDelimiter;			// Delimiter for the serial stream

			bool			mBinary;			// Flag if in binary mode
			bool			mBlocking;			// Flag if a blocking setup
#ifdef WIN32
			HANDLE			mFD;				// Windows Handler
#elif defined __linux__
			int32_t			mFD;				// Linux Handler
#endif
		};
	} // End Namespace Communications
} // End Namespace Essentials

#endif // CPP_SERIAL