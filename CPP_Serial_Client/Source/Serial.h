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
#else

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
			int8_t FlushInput();
			int8_t FlushOutput();
			int8_t Write();
			int8_t WriteBreak();
			int8_t Close();

			/// <summary>Sets the port to connect on</summary>
			/// <param name="port"> -[in]- Port to connect on.</param>
			/// <returns>0 if successful, -1 if fails. Call GetLastError to find out more.</returns>
			int8_t SetPort(const std::string port);

			/// <summary>Sets the baudrate for the serial port</summary>
			/// <param name="baud"> -[in]- Baudrate for the port.</param>
			/// <returns>0 if successful, -1 if fails. Call GetLastError to find out more.</returns>
			int8_t SetBaudrate(const BaudRate baud);

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
			int8_t SetTimeout(const int16_t timeoutMS);

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

			// GETTERS
			std::string GetPort();
			BaudRate GetBaudrate();
			Parity GetParity();
			ByteSize GetByteSize();
			int16_t GetTimeout();
			StopBits GetStopBits();
			FlowControl GetFlowControl();
			uint8_t GetDelimiter();
			int8_t GetCTS();
			int8_t GetDSR();
			int8_t GetRI();
			int8_t GetCD();
			bool GetBinary();
			int8_t GetInQueueLength();

			std::string GetLastError();

		protected:
		private:
			std::string		mPort;
			BaudRate		mBaudRate;
			Parity			mParity;
			ByteSize		mByteSize;
			int16_t			mTimeout;
			StopBits		mStopBits;
			FlowControl		mFlowControl;
			SerialError		mLastError;
			uint8_t			mDelimiter;

			bool			mIsOpen;
			bool			mBinary;

			int32_t			mFD;

		};
	} // End Namespace Communications
} // End Namespace Essentials

#endif // CPP_SERIAL