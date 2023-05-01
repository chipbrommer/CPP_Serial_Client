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
			mBaudRate = BaudRate::BAUDRATE_INVALID;
			mByteSize = ByteSize::INVALID;
			mTimeout = -1;
			mParity = Parity::INVALID;
			mStopBits = StopBits::ONE;
			mFlowControl = FlowControl::HARDWARE;
			mLastError = SerialError::NONE;
			mDelimiter = 0x00;
			mIsOpen = false;
			mBinary = false;
#ifdef WIN32
			mFD = INVALID_HANDLE_VALUE;
#elif defined LINUX
			mFD = -1;
#endif
		}

		Serial::Serial(const std::string port, const BaudRate baud, const ByteSize bytes, const Parity parity)
		{
			mPort = port;
			mBaudRate = baud;
			mByteSize = bytes;
			mParity = parity;

			mTimeout = -1;
			mStopBits = StopBits::ONE;
			mFlowControl = FlowControl::HARDWARE;
			mLastError = SerialError::NONE;
			mDelimiter = 0x00;
			mIsOpen = false;
			mBinary = false;
#ifdef WIN32
			mFD = INVALID_HANDLE_VALUE;
#elif defined LINUX
			mFD = -1;
#endif
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
			if(SetSerialPort(port) < 0)
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
			// Run through a series of checks to verify everything is ready.
			if(mIsOpen)
			{
				mLastError = SerialError::SERIAL_PORT_ALREADY_OPEN;
				return -1;
			}

			if (mPort == "")
			{
				mLastError = SerialError::PORT_NOT_SET;
				return -1;
			}

			if (mBaudRate == BaudRate::BAUDRATE_INVALID)
			{
				mLastError = SerialError::BAUDRATE_NOT_SET;
				return -1;
			}

			if (mByteSize == ByteSize::INVALID)
			{
				mLastError = SerialError::BYTESIZE_NOT_SET;
				return -1;
			}

			if (mParity == Parity::INVALID)
			{
				mLastError = SerialError::PARITY_NOT_SET;
				return -1;
			}

			if (mStopBits == StopBits::INVALID)
			{
				mLastError = SerialError::STOPBITS_NOT_SET;
				return -1;
			}

#ifdef WIN32
			if (mBlocking)
			{

			}
			else
			{

			}
#elif defined LINUX
			if (mBlocking)
			{

			}
#endif
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

		int8_t Serial::Flush()
		{
			if (!mIsOpen)
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

			int rtn = 0;

#ifdef WIN32
			if (mFD != INVALID_HANDLE_VALUE)
			{
				rtn = FlushFileBuffers(mFD);
			}

			// FlushFileBuffers returns 0 on fail, adjust to -1;
			if (rtn == 0)
			{
				rtn = -1;
			}

#elif defined LINUX
			if (mFD >= 0)
			{
				rtn = tcFlush(mFD, TCIFLUSH);
			}
#endif
			if (rtn == -1)
			{
				mLastError = SerialError::FLUSH_ERROR;
			}

			return rtn;
		}

		int8_t Serial::FlushInput()
		{
			if (!mIsOpen)
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

#ifdef WIN32
			PurgeComm(mFD, PURGE_RXCLEAR);
#elif defined LINUX
			if (mFD >= 0)
			{
				tcFlush(mFD, TCIFLUSH);
			}
#endif
			return 0;
		}

		int8_t Serial::FlushOutput()
		{
			if (!mIsOpen)
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

#ifdef WIN32
			PurgeComm(mFD, PURGE_TXCLEAR);
#elif defined LINUX
			if (mFD >= 0)
			{
				tcFlush(mFD, TCIFLUSH);
			}
#endif
			return 0;
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
			// Holds value returned from closing
			int result;

#ifdef WIN32
			result = CloseHandle(mFD);
#elif defined LINUX
			result = close(m_status);
#endif

			// Check if error in closing
			if (result < 0)
			{
				mLastError = SerialError::CLOSE_FAILURE;
				return -1;
			}

			// return success
			return 0;
		}

		int8_t Serial::SetSerialPort(const std::string port)
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

		int8_t Serial::SetTimeout(const int16_t timeoutMS)
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

		int8_t Serial::SetDelimiter(const uint8_t delimiter)
		{
			mDelimiter = delimiter;

			if (mDelimiter == delimiter)
			{
				return 0;
			}

			mLastError = SerialError::DELIMITER_SET_FAILURE;
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

		std::string Serial::GetPort()
		{
			return mPort;
		}

		BaudRate Serial::GetBaudrate()
		{
			return mBaudRate;
		}

		Parity Serial::GetParity()
		{
			return mParity;
		}

		ByteSize Serial::GetByteSize()
		{
			return mByteSize;
		}

		int16_t Serial::GetTimeout()
		{
			return mTimeout;
		}

		StopBits Serial::GetStopBits()
		{
			return mStopBits;
		}

		FlowControl Serial::GetFlowControl()
		{
			return mFlowControl;
		}

		uint8_t Serial::GetDelimiter()
		{
			return mDelimiter;
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

		bool Serial::GetBinary()
		{
			return mBinary;
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