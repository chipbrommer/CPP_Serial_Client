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
			mTimeout = 0;
			mParity = Parity::INVALID;
			mStopBits = StopBits::ONE;
			mFlowControl = FlowControl::HARDWARE;
			mLastError = SerialError::NONE;
			mDelimiter = 0x00;
			mBinary = false;
#ifdef WIN32
			mFD = INVALID_HANDLE_VALUE;
#elif defined __linux__
			mFD = -1;
#endif
		}

		Serial::Serial(const std::string port, const BaudRate baud, const ByteSize bytes, const Parity parity)
		{
			mPort = port;
			mBaudRate = baud;
			mByteSize = bytes;
			mParity = parity;

			mTimeout = 0;
			mStopBits = StopBits::ONE;
			mFlowControl = FlowControl::HARDWARE;
			mLastError = SerialError::NONE;
			mDelimiter = 0x00;
			mBinary = false;
#ifdef WIN32
			mFD = INVALID_HANDLE_VALUE;
#elif defined __linux__
			mFD = -1;
#endif
		}

		Serial::~Serial()
		{
			if (IsOpen())
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
			if(IsOpen())
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
			// open connection to serial port
			mFD = CreateFileA(mPort.c_str(),			// Serial Port Name
				GENERIC_READ | GENERIC_WRITE,			// Access general read/write 
				0,										// Do not share access
				NULL,									// No Security Attributes
				OPEN_EXISTING,							// Open Existing File
				FILE_ATTRIBUTE_NORMAL,					// Normal file attributes
				NULL);									// Do not use template

			// Connection check : throw error if invalid handle value
			if (mFD == INVALID_HANDLE_VALUE)
			{
				mLastError = SerialError::HANDLE_SETUP_FAILURE;
				return -1;
			}

			// Setup Settings for Serial Port
			DCB serialSettings = { 0 };
			serialSettings.DCBlength = sizeof(serialSettings);
			GetCommState(mFD, &serialSettings);
			serialSettings.BaudRate = (DWORD)mBaudRate;
			serialSettings.ByteSize = 8;
			serialSettings.StopBits = ONESTOPBIT;
			serialSettings.Parity = NOPARITY;
			// Save settings, check for error
			if (SetCommState(mFD, &serialSettings) == 0)
			{
				mLastError = SerialError::SET_COMMSTATE_FAILURE;
				return -1;
			}

			// Setup Timeouts for Serial Port
			COMMTIMEOUTS serialTimeout = { 0 };
			serialTimeout.ReadIntervalTimeout = mTimeout;

			// Save timeouts, check for error
			if (SetCommTimeouts(mFD, &serialTimeout) == 0)
			{
				mLastError = SerialError::SET_COMMTIMEOUT_FAILURE;
				return -1;
			}
#elif defined __linux__
			if (mBlocking)
			{

			}
#endif
			return -1;
		}

		bool Serial::IsOpen()
		{
#ifdef WIN32
			return mFD != INVALID_HANDLE_VALUE;
#elif defined __linux__
			return mFD != -1;
#endif
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
			if (!IsOpen())
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

#elif defined __linux__
			if (mFD >= 0)
			{
				rtn = tcflush(mFD, TCIFLUSH);
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
			if (!IsOpen())
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

#ifdef WIN32
			PurgeComm(mFD, PURGE_RXCLEAR);
#elif defined __linux__
			if (mFD >= 0)
			{
				tcflush(mFD, TCIFLUSH);
			}
#endif
			return 0;
		}

		int8_t Serial::FlushOutput()
		{
			if (!IsOpen())
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

#ifdef WIN32
			PurgeComm(mFD, PURGE_TXCLEAR);
#elif defined __linux__
			if (mFD >= 0)
			{
				tcflush(mFD, TCIFLUSH);
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

			// Close Handle returns 0 for failure, adjust
			if (result == 0)
			{
				result = -1;
			}
#elif defined __linux__
			result = close(mFD);
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

		int8_t Serial::SetTimeout(const uint32_t timeoutMS)
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

		uint32_t Serial::GetTimeout()
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

		int32_t Serial::GetInQueueLength()
		{
			if (!IsOpen())
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}
#ifdef WIN32
			COMSTAT cs;
			if (ClearCommError(mFD, NULL, &cs)) 
			{
				return cs.cbInQue;
			}
#elif defined __linux__
			int32_t inQueue = -1;

			if (ioctl(mFD, FIONREAD, &inQueue) >= 0)
			{
				return inQueue;
			}
#endif
			mLastError = SerialError::QUEUE_LENGTH_READ_FAILURE;
			return -1;
		}

		std::string Serial::GetLastError()
		{
			return SerialErrorMap[mLastError];
		}

	} // End Namespace Communications
} // End Namespace Essentials