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
			/// <param name="port">Port to connect on.</param>
			/// <param name="baud">Baudrate for the port.</param>
			/// <param name="bytes">BytesSize for the port.</param>
			/// <param name="parity">Parity for the port.</param>
			Serial(const std::string port, const BaudRate baud, const ByteSize bytes, const Parity parity);

			/// <summary>Default Deconstructor</summary>
			~Serial();

			/// <summary>Initializer that accepts some default parameters if the default constructor is used.</summary>
			/// <param name="port">Port to connect on.</param>
			/// <param name="baud">Baudrate for the port.</param>
			/// <param name="parity">Parity for the port.</param>
			/// <returns>0 if successful, -1 if fails. Call GetLastError to find out more.</returns>
			int8_t Configure(std::string port, BaudRate baud, Parity parity);

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

			// SETTERS
			int8_t SetPort(std::string port);
			int8_t SetBaudrate(BaudRate baud);
			int8_t SetParity(Parity patiry);
			int8_t SetByteSize(ByteSize size);
			int8_t SetTimeout();
			int8_t SetStopBits(StopBits bits);
			int8_t SetFlowControl(FlowControl flow);
			int8_t SetDelimiter();
			int8_t SetBreak();
			int8_t SetRTS();
			int8_t SetDTR();
			int8_t SetBinary();

			// GETTERS
			int8_t GetPort();
			int8_t GetBaudrate();
			int8_t GetParity();
			int8_t GetByteSize();
			int8_t GetTimeout();
			int8_t GetStopBits();
			int8_t GetFlowControl();
			int8_t GetDelimiter();
			int8_t GetCTS();
			int8_t GetDSR();
			int8_t GetRI();
			int8_t GetCD();
			int8_t GetBinary();
			int8_t GetInQueueLength();

			std::string GetLastError();

		protected:
		private:
			std::string		mPort;
			BaudRate		mBaudRate;
			ByteSize		mByteSize;
			Parity			mParity;
			StopBits		mStopBits;
			FlowControl		mFlowControl;
			SerialError			mLastError;

			bool			mIsOpen;
			bool			mBinary;

			int32_t			mFD;

		};
	} // End Namespace Communications
} // End Namespace Essentials

#endif // CPP_SERIAL