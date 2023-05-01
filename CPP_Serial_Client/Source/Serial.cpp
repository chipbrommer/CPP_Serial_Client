///////////////////////////////////////////////////////////////////////////////
//!
//! @file		Serial.cpp
//! 
//! @brief		Implementation of a serial class
//! 
//! @author		Chip Brommer
//! 
//! @date		< 04 / 30 / 2023 > Initial Start Date
//!
/*****************************************************************************/

///////////////////////////////////////////////////////////////////////////////
//
//  Includes:
//          name                        reason included
//          --------------------        ---------------------------------------
#include	"Serial.h"					// Serial Class
//
///////////////////////////////////////////////////////////////////////////////

namespace Essentials
{
	namespace Communications
	{
		Serial::Serial()
		{
			mPort = "";
			mBaudRate = BaudRate::BAUD_INVALID;
			mByteSize = ByteSize::EIGHT;
			mParity = Parity::NONE;
			mStopBits = StopBits::ONE;
			mFlowControl = FlowControl::HARDWARE;
			mLastError = SerialError::NONE;
			mIsOpen = false;
			mBinary = false;
			mFD = -1;
		}

		Serial::Serial(const std::string port, const BaudRate baud, const ByteSize bytes, const Parity parity)
		{
			mPort = port;
			mBaudRate = baud;
			mByteSize = bytes;
			mParity = parity;

			mStopBits = StopBits::ONE;
			mFlowControl = FlowControl::HARDWARE;
			mLastError = SerialError::NONE;
			mIsOpen = false;
			mBinary = false;
			mFD = -1;
		}

		Serial::~Serial()
		{

		}

		int8_t Serial::Configure(std::string port, BaudRate baud, Parity parity)
		{
			return -1;
		}

		int8_t Serial::Open()
		{
			return -1;
		}

		bool Serial::IsOpen()
		{
			return mIsOpen;
		}

		int8_t Serial::WaitReadable()
		{
			return -1;
		}

		int8_t Serial::Read()
		{
			return -1;
		}

		int8_t Serial::ReadLine()
		{
			return -1;
		}

		int8_t Serial::FlushInput()
		{
			return -1;
		}

		int8_t Serial::FlushOutput()
		{
			return -1;
		}

		int8_t Serial::Write()
		{
			return -1;
		}

		int8_t Serial::WriteBreak()
		{
			return -1;
		}

		int8_t Serial::Close()
		{
			return -1;
		}

		int8_t Serial::SetPort(std::string port)
		{
			return -1;
		}

		int8_t Serial::SetBaudrate(BaudRate baud)
		{
			return -1;
		}

		int8_t Serial::SetParity(Parity patiry)
		{
			return -1;
		}

		int8_t Serial::SetByteSize(ByteSize size)
		{
			return -1;
		}

		int8_t Serial::SetTimeout()
		{
			return -1;
		}

		int8_t Serial::SetStopBits(StopBits bits)
		{
			return -1;
		}

		int8_t Serial::SetFlowControl(FlowControl flow)
		{
			return -1;
		}

		int8_t Serial::SetDelimiter()
		{
			return -1;
		}

		int8_t Serial::SetBreak()
		{
			return -1;
		}

		int8_t Serial::SetRTS()
		{
			return -1;
		}

		int8_t Serial::SetDTR()
		{
			return -1;
		}

		int8_t Serial::SetBinary()
		{
			return -1;
		}

		int8_t Serial::GetPort()
		{
			return -1;
		}

		int8_t Serial::GetBaudrate()
		{
			return -1;
		}

		int8_t Serial::GetParity()
		{
			return -1;
		}

		int8_t Serial::GetByteSize()
		{
			return -1;
		}

		int8_t Serial::GetTimeout()
		{
			return -1;
		}

		int8_t Serial::GetStopBits()
		{
			return -1;
		}

		int8_t Serial::GetFlowControl()
		{
			return -1;
		}

		int8_t Serial::GetDelimiter()
		{
			return -1;
		}

		int8_t Serial::GetCTS()
		{
			return -1;
		}

		int8_t Serial::GetDSR()
		{
			return -1;
		}

		int8_t Serial::GetRI()
		{
			return -1;
		}

		int8_t Serial::GetCD()
		{
			return -1;
		}

		int8_t Serial::GetBinary()
		{
			return -1;
		}

		int8_t Serial::GetInQueueLength()
		{
			return -1;
		}

		std::string Serial::GetLastError()
		{
			return SerialErrorMap[mLastError];
		}

	} // End Namespace Communications
} // End Namespace Essentials