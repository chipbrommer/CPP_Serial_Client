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
	enum class BaudRate : uint8_t
	{
		BAUD_INVALID,
	};

	enum class ByteSize : uint8_t
	{
		Five = 5,
		Six = 6,
		Seven = 7,
		Eight = 8,
	};

	enum class Parity : uint8_t
	{
		None,
		Odd,
		Even,
		Mark,
		Space,
	};

	enum class StopBits : uint8_t
	{
		One,
		Two,
		OnePointFive,
	};

	enum class FlowControl : uint8_t
	{
		None,
		Software,
		Hardware,
	};

	class Serial
	{
	public:
		Serial();
		~Serial();
		int8_t Initialize();
		int8_t Open();
		int8_t IsOpen();
		int8_t WaitReadable();
		int8_t Read();
		int8_t ReadLine();
		int8_t FlushInput();
		int8_t FlushOutput();
		int8_t Write();
		int8_t WriteBreak();
		int8_t Close();

		// SETTERS
		int8_t SetPort();
		int8_t SetBaudrate();
		int8_t SetParity();
		int8_t SetByteSize();
		int8_t SetTimeout();
		int8_t SetStopBits();
		int8_t SetFlowControl();
		int8_t SetDelimiter();
		int8_t SetBreak();
		int8_t SetRTS();
		int8_t SetDTR();

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
		int8_t GetInQueueLength();

	protected:
	private:
		std::string		mPort;
		BaudRate		mBaudRate;
		ByteSize		mByteSize;
		Parity			mParity;
		StopBits		mStopBits;
		FlowControl		mFlowControl;
	};
}

#endif // CPP_SERIAL