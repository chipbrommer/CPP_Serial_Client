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
			serialSettings.ByteSize = mByteSize;
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
			// Open connection to serial port
			// Call open + send port name and Read Write access + Program doesnt control access + Send without restrictions
			if (mBlocking)
			{
				mFD = open(mPort.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_APPEND);
			}
			else
			{
				mFD = open(mPort.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_APPEND | O_NONBLOCK);
			}

			// Handle check : -1 = open failed
			if (mFD < 0)
			{
				mLastError = SerialError::SERIAL_LINUX_OPEN_FAILURE;
				return -1;
			}

			// Get the current serial attriutes, if tcgetattr returns -1, print error and return
			termios term;
			if (tcgetattr(mFD, &term) < 0)
			{
				mLastError = SerialError::SERIAL_LINUX_GETATTRIBUTES_FAILURE;
				return -1;
			}
			else 
			{
				// Setup settings for Serial Port communication
				speed_t baud = 0;

				switch (mBaudRate)
				{
				case BaudRate::BAUDRATE_50:		baud = B50;
				case BaudRate::BAUDRATE_75:		baud = B75;
				case BaudRate::BAUDRATE_110:	baud = B110;
				case BaudRate::BAUDRATE_134:	baud = B134;
				case BaudRate::BAUDRATE_150:	baud = B150;
				case BaudRate::BAUDRATE_200:	baud = B200;
				case BaudRate::BAUDRATE_300:	baud = B300;
				case BaudRate::BAUDRATE_600:	baud = B600;
				case BaudRate::BAUDRATE_1200:	baud = B1200;
				case BaudRate::BAUDRATE_2400:	baud = B2400;
				case BaudRate::BAUDRATE_4800:	baud = B4800;
				case BaudRate::BAUDRATE_9600:	baud = B9600;
				case BaudRate::BAUDRATE_19200:	baud = B19200;
				case BaudRate::BAUDRATE_38400:	baud = B38400;
				case BaudRate::BAUDRATE_57600:	baud = B57600;
				case BaudRate::BAUDRATE_115200: baud = B115200;
				case BaudRate::BAUDRATE_460800: baud = B460800;
				case BaudRate::BAUDRATE_921600: baud = B921600;
				default: baud = B9600;
					// TODO - add custom bauds for windows only ones. 
				}

				cfsetospeed(&term, baud);		// set the baudrate from "mBaudRate" parameter
				cfsetispeed(&term, baud);

				switch (mByteSize)
				{
				case ByteSize::FIVE:	term.c_cflag |= CS5;
				case ByteSize::SIX:		term.c_cflag |= CS6;
				case ByteSize::SEVEN:	term.c_cflag |= CS7;
				case ByteSize::EIGHT: // Intentional fall through
				default:				term.c_cflag |= CS8;
				}

				// TODO - parity, stopbits, flow control.

				term.c_cflag &= ~PARENB;				// Clear parity bit, disabling parity
				term.c_cflag &= ~CSTOPB;				// Use one stop bit
				term.c_cflag &= ~CRTSCTS;				// Disable RTS/CTS hardware flow control
				term.c_cflag |= CREAD | CLOCAL;			// Turn on READ & ignore ctrl lines (CLOCAL = 1)
				term.c_lflag &= ~ICANON;				// Disable canonical
				term.c_lflag &= ~ECHO;					// Disable echo
				term.c_lflag &= ~ECHOE;					// Disable erasure
				term.c_lflag &= ~ECHONL;				// Disable new-line echo
				term.c_lflag &= ~ISIG;					// Disable interpretation of INTR, QUIT and SUSP
				term.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
				term.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes
				term.c_oflag &= ~OPOST;					// Prevent special interpretation of output bytes
				term.c_oflag &= ~ONLCR;					// Prevent conversion of newline
				term.c_cc[VTIME] = mBlocking ? 1 : 0;	// 0 = Return as soon as any data is received.
				term.c_cc[VMIN] = mBlocking ? cc_t(1) : 0;

				// Save settings, check for error
				if (tcsetattr(mFD, TCSANOW, &term) < 0)
				{
					mLastError = SerialError::SERIAL_LINUX_SETATTRIBUTES_FAILURE;
					return -1;
				}
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
				rtn = tcflush(mFD, TCIOFLUSH);
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
				tcflush(mFD, TCOFLUSH);
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

		int8_t Serial::SetBaudrate(const BaudRate baud, const uint32_t custom)
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

		int8_t SetCustomBaudrate()
		{
			return -1;
		}

	} // End Namespace Communications
} // End Namespace Essentials