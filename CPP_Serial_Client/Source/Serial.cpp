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
			mByteSize = ByteSize::INVALID;
			mParity = Parity::INVALID;
			mStopBits = StopBits::INVALID;
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
			if (mIsOpen)
			{
				Close();
			}
		}

		int8_t Serial::Configure(const std::string port, const BaudRate baud, const ByteSize bytes, const Parity parity)
		{
			if(SetPort(port) < 0)
			{
				return -1;
			}

			if (SetBaudrate(baud) < 0)
			{
				return -1;
			}

			if (SetByteSize(bytes) < 0)
			{
				return -1;
			}

			if (SetParity(parity) < 0)
			{
				return -1;
			}

			return 0;
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

		int8_t Serial::SetPort(const std::string port)
		{
			mPort = port;

			if (mPort == port)
			{
				return 0;
			}

			mLastError = SerialError::PORT_SET_FAILURE;
			return -1;
		}

		int8_t Serial::SetBaudrate(const BaudRate baud)
		{
			mBaudRate = baud;

			if (mBaudRate == baud)
			{
				return 0;
			}

			mLastError = SerialError::BAUDRATE_SET_FAILURE;
			return -1;
		}

		int8_t Serial::SetParity(const Parity parity)
		{
			mParity = parity;

			if (mParity == parity)
			{
				return 0;
			}

			mLastError = SerialError::PARITY_SET_FAILURE;
			return -1;
		}

		int8_t Serial::SetByteSize(const ByteSize size)
		{
			mByteSize = size;

			if (mByteSize == size)
			{
				return 0;
			}

			mLastError = SerialError::BYTESIZE_SET_FAILURE;
			return -1;
		}

		int8_t Serial::SetTimeout(const uint16_t timeoutMS)
		{
			mTimeout = timeoutMS;

			if (mTimeout == timeoutMS)
			{
				return 0;
			}

			mLastError = SerialError::TIMEOUT_SET_FAILURE;
			return -1;
		}

		int8_t Serial::SetStopBits(const StopBits bits)
		{
			mStopBits = bits;

			if (mStopBits == bits)
			{
				return 0;
			}

			mLastError = SerialError::STOPBITS_SET_FAILURE;
			return -1;
		}

		int8_t Serial::SetFlowControl(const FlowControl flow)
		{
			mFlowControl = flow;

			if (mFlowControl == flow)
			{
				return 0;
			}

			mLastError = SerialError::FLOWCONTROL_SET_FAILURE;
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