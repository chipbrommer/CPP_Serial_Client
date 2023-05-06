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
			mTitle = "Serial";
			mPort = "";
			mBaudRate = BaudRate::BAUDRATE_INVALID;
			mCustomBaudRate = -1;
			mByteSize = ByteSize::INVALID;
			mTimeout = 0;
			mParity = Parity::INVALID;
			mStopBits = StopBits::ONE;
			mFlowControl = FlowControl::HARDWARE;
			mLastError = SerialError::NONE;
			mDelimiter = "\n";
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

			mTitle = "Serial_" + mPort;
			mCustomBaudRate = -1;
			mTimeout = 0;
			mStopBits = StopBits::ONE;
			mFlowControl = FlowControl::HARDWARE;
			mLastError = SerialError::NONE;
			mDelimiter = "\n";
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
			// Validity Check for main elements 
			if (!ValidityCheck())
			{
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
#endif
			// Configure the port to the set parameters. 
			if (ReconfigurePort() < 0)
			{
				return -1;
			}

			return 0;
		}

		int8_t Serial::ReconfigurePort()
		{
			// Validity Check for main elements 
			if (!ValidityCheck())
			{
				return -1;
			}

#ifdef WIN32
			// Connection check : throw error if invalid handle value
			if (mFD == INVALID_HANDLE_VALUE)
			{
				mLastError = SerialError::BAD_HANDLE;
				return -1;
			}

			// Setup Settings for Serial Port
			DCB serialSettings = { 0 };
			serialSettings.DCBlength = sizeof(serialSettings);
			GetCommState(mFD, &serialSettings);

			DWORD baud = 0;

			switch (mBaudRate)
			{
			case BaudRate::BAUDRATE_50:		serialSettings.BaudRate = 50;
			case BaudRate::BAUDRATE_75:		serialSettings.BaudRate = 75;
			case BaudRate::BAUDRATE_110:	serialSettings.BaudRate = CBR_110;
			case BaudRate::BAUDRATE_134:	serialSettings.BaudRate = 134;
			case BaudRate::BAUDRATE_150:	serialSettings.BaudRate = 150;
			case BaudRate::BAUDRATE_200:	serialSettings.BaudRate = 200;
			case BaudRate::BAUDRATE_300:	serialSettings.BaudRate = CBR_300;
			case BaudRate::BAUDRATE_600:	serialSettings.BaudRate = CBR_600;
			case BaudRate::BAUDRATE_1200:	serialSettings.BaudRate = CBR_1200;
			case BaudRate::BAUDRATE_2400:	serialSettings.BaudRate = CBR_2400;
			case BaudRate::BAUDRATE_4800:	serialSettings.BaudRate = CBR_4800;
			case BaudRate::BAUDRATE_9600:	serialSettings.BaudRate = CBR_9600;
			case BaudRate::BAUDRATE_14400:	serialSettings.BaudRate = CBR_14400;
			case BaudRate::BAUDRATE_19200:	serialSettings.BaudRate = CBR_19200;
			case BaudRate::BAUDRATE_38400:	serialSettings.BaudRate = CBR_38400;
			case BaudRate::BAUDRATE_57600:	serialSettings.BaudRate = CBR_57600;
			case BaudRate::BAUDRATE_115200: serialSettings.BaudRate = CBR_115200;
			case BaudRate::BAUDRATE_128000: serialSettings.BaudRate = CBR_128000;
			case BaudRate::BAUDRATE_256000: serialSettings.BaudRate = CBR_256000;
			case BaudRate::BAUDRATE_460800: serialSettings.BaudRate = 460800;
			case BaudRate::BAUDRATE_921600: serialSettings.BaudRate = 921600;
			case BaudRate::BAUDRATE_CUSTOM: serialSettings.BaudRate = mCustomBaudRate;
			default:						serialSettings.BaudRate = CBR_9600;
			}

			switch (mByteSize)
			{
			case ByteSize::FIVE:	serialSettings.ByteSize = 5;
			case ByteSize::SIX:		serialSettings.ByteSize = 6;
			case ByteSize::SEVEN:	serialSettings.ByteSize = 7;
			default:	// Intentional fall through.
			case ByteSize::EIGHT:	serialSettings.ByteSize = 8;
			}

			switch (mParity)
			{
			case Parity::ODD:		serialSettings.Parity = ODDPARITY;
			case Parity::EVEN:		serialSettings.Parity = EVENPARITY;
			case Parity::MARK:		serialSettings.Parity = MARKPARITY;
			case Parity::SPACE:		serialSettings.Parity = SPACEPARITY;
			default:	// Intentional fall through.
			case Parity::NONE:		serialSettings.Parity = NOPARITY;
			}

			switch (mStopBits)
			{
			case StopBits::TWO:			serialSettings.StopBits = TWOSTOPBITS;
			case StopBits::ONE_FIVE:;	serialSettings.StopBits = ONE5STOPBITS;
			default:	// Intentional fall through.
			case StopBits::ONE:			serialSettings.StopBits = ONESTOPBIT;
			}

			switch (mFlowControl)
			{
			case FlowControl::SOFTWARE:
			{
				serialSettings.fOutxCtsFlow = false;
				serialSettings.fRtsControl = RTS_CONTROL_DISABLE;
				serialSettings.fOutX = true;
				serialSettings.fInX = true;
			}
			break;
			case FlowControl::HARDWARE:
			{
				serialSettings.fOutxCtsFlow = true;
				serialSettings.fRtsControl = RTS_CONTROL_HANDSHAKE;
				serialSettings.fOutX = false;
				serialSettings.fInX = false;
			}
			break;
			default:	// Intentional fall through. 
			case FlowControl::NONE:
			{
				serialSettings.fOutxCtsFlow = false;
				serialSettings.fRtsControl = RTS_CONTROL_DISABLE;
				serialSettings.fOutX = false;
				serialSettings.fInX = false;
			}
			break;
			}

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
			// Handle check : -1 = open failed
			if (mFD < 0)
			{
				mLastError = SerialError::LINUX_BAD_HANDLE;
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
				case BaudRate::BAUDRATE_50:		baud = B50;			break;
				case BaudRate::BAUDRATE_75:		baud = B75;			break;
				case BaudRate::BAUDRATE_110:	baud = B110;		break;
				case BaudRate::BAUDRATE_134:	baud = B134;		break;
				case BaudRate::BAUDRATE_150:	baud = B150;		break;
				case BaudRate::BAUDRATE_200:	baud = B200;		break;
				case BaudRate::BAUDRATE_300:	baud = B300;		break;
				case BaudRate::BAUDRATE_600:	baud = B600;		break;
				case BaudRate::BAUDRATE_1200:	baud = B1200;		break;
				case BaudRate::BAUDRATE_2400:	baud = B2400;		break;
				case BaudRate::BAUDRATE_4800:	baud = B4800;		break;
				case BaudRate::BAUDRATE_9600:	baud = B9600;		break;
				case BaudRate::BAUDRATE_19200:	baud = B19200;		break;
				case BaudRate::BAUDRATE_38400:	baud = B38400;		break;
				case BaudRate::BAUDRATE_57600:	baud = B57600;		break;
				case BaudRate::BAUDRATE_115200: baud = B115200;		break;
				case BaudRate::BAUDRATE_460800: baud = B460800;		break;
				case BaudRate::BAUDRATE_921600: baud = B921600;		break;
				default: baud = B9600;
					// TODO - add custom bauds for windows only ones. 
				}

				cfsetospeed(&term, baud);		// set the baudrate from "mBaudRate" parameter
				cfsetispeed(&term, baud);

				switch (mByteSize)
				{
				case ByteSize::FIVE:	term.c_cflag |= CS5;		break;
				case ByteSize::SIX:		term.c_cflag |= CS6;		break;
				case ByteSize::SEVEN:	term.c_cflag |= CS7;		break;
				case ByteSize::EIGHT: // Intentional fall through
				default:				term.c_cflag |= CS8;
				}

				switch (mParity)
				{
				case Parity::NONE:		term.c_cflag &= ~(PARENB | PARODD);	break;
				case Parity::ODD:		term.c_cflag |= (PARENB | PARODD);	break;
				case Parity::EVEN:		term.c_cflag &= ~PARODD; term.c_cflag |= PARENB; break;
				case Parity::MARK:		// Intnetional fall through		
				case Parity::SPACE:		// Intnetional fall through	
				default:				term.c_cflag &= ~PARENB;
				}

				switch (mStopBits)
				{
				case StopBits::ONE:			term.c_cflag &= ~CSTOPB;	break;
				case StopBits::TWO:		// Intnetional fall through, 1.5 not supported in linux so we use two.
				case StopBits::ONE_FIVE:;	term.c_cflag &= CSTOPB;		break;
				default:					term.c_cflag &= ~CSTOPB;
				}

				switch (mFlowControl)
				{
				case FlowControl::NONE:			term.c_cflag &= ~CRTSCTS;	term.c_iflag &= ~(IXON | IXOFF | IXANY);	break;
				case FlowControl::SOFTWARE:;	term.c_cflag &= ~CRTSCTS;	term.c_iflag |= (IXON | IXOFF | IXANY);		break;
				case FlowControl::HARDWARE:;	term.c_cflag |= CRTSCTS;	term.c_iflag &= ~(IXON | IXOFF | IXANY);	break;
				default:						term.c_cflag &= ~CRTSCTS;
				}

				term.c_cflag &= ~CRTSCTS;				// Disable RTS/CTS hardware flow control
				term.c_cflag |= CREAD | CLOCAL;			// Turn on READ & ignore ctrl lines (CLOCAL = 1)
				term.c_lflag &= ~ICANON;				// Disable canonical
				term.c_lflag &= ~ECHO;					// Disable echo
				term.c_lflag &= ~ECHOE;					// Disable erasure
				term.c_lflag &= ~ECHONL;				// Disable new-line echo
				term.c_lflag &= ~ISIG;					// Disable interpretation of INTR, QUIT and SUSP
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
			// Return success. 
			return 0;
		}

		bool Serial::IsOpen()
		{
#ifdef WIN32
			return mFD != INVALID_HANDLE_VALUE;
#elif defined __linux__
			return mFD != -1;
#endif
		}

		int32_t Serial::Read(void* buffer, const uint32_t size)
		{
			// TODO - implement timeout

			// Check mFD first
			if (!IsOpen())
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

			// Holds value returned from reading
			int32_t rtn = -1;

			// Read data from the buffer 
#ifdef WIN32
			DWORD numRead = 0;
			rtn = ReadFile(mFD, &buffer, size, &numRead, NULL);

			// ReadFile returns 0 on fail, adjust to -1;
			if (rtn == 0)
			{
				rtn = -1;
			}
			else
			{
				rtn = numRead;
			}
#elif defined __linux__
			rtn = write(mFD, buffer, size);
#endif

			// Check if read was successful
			if (rtn < 0)
			{
				mLastError = SerialError::READ_FAILURE;
				return -1;
			}

			// Return result
			return rtn;
		}

		int32_t Serial::ReadLine(void* buffer, const uint32_t size, std::string delimiter)
		{
			// Verify port is open. 
			if (!IsOpen())
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

			// If parameter isnt passed in and delimiter is set, use it instead
			if (delimiter == "\n" && (mDelimiter != "\n" && !mDelimiter.empty()) )
			{
				delimiter = mDelimiter;
			}
			else if (delimiter.empty())
			{
				delimiter = "\n";
			}

			// Catch 0 for size
			if (size <= 0)
			{
				return -1;
			}

			// Hold delimiter length.
			uint32_t delimLength = static_cast<uint32_t>(delimiter.size());

			uint8_t* inBuffer = static_cast<uint8_t*>(alloca(size * sizeof(uint8_t)));
			uint32_t totalRead = 0;
			int32_t numRead = -1;

			// Read one byte at a time until delimiter is found or size cap is hit. 
			for(;;)
			{
				numRead = Read(inBuffer + totalRead, 1);

				if (numRead == -1)
				{
					break;
				}

				totalRead += numRead;

				if (numRead == 0)
				{
					break; // Timeout occured on reading 1 byte
				}

				if (totalRead < delimLength)
				{
					continue;
				}

				if (std::string(reinterpret_cast<const char*>(inBuffer + totalRead - delimLength), delimLength) == delimiter) 
				{
					break; // EOL found
				}

				if (totalRead == size)
				{
					break; // Reached the maximum read length
				}
			}

			// Check if read was successful
			if (totalRead == 0 && numRead == -1)
			{
				mLastError = SerialError::READ_LINE_FAILURE;
				return -1;
			}

			// Copy the buffer contents and return total read
			memcpy(buffer, inBuffer, totalRead);
			return totalRead;
		}

		int8_t Serial::Flush()
		{
			if (!IsOpen())
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

			uint8_t rtn = 0;

#ifdef WIN32
			rtn = FlushFileBuffers(mFD);

			// FlushFileBuffers returns 0 on fail, adjust to -1;
			if (rtn == 0)
			{
				rtn = -1;
			}
#elif defined __linux__
			rtn = tcflush(mFD, TCIOFLUSH);
#endif
			if (rtn == -1)
			{
				mLastError = SerialError::FLUSH_IO_ERROR;
			}

			return 0;
		}

		int8_t Serial::FlushInput()
		{
			if (!IsOpen())
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

			uint8_t rtn = 0;

#ifdef WIN32
			if (mFD != INVALID_HANDLE_VALUE)
			{
				rtn = PurgeComm(mFD, PURGE_RXCLEAR);
			}

			// PurgeComm returns 0 on fail, adjust to -1;
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
				mLastError = SerialError::FLUSH_INPUT_ERROR;
			}

			return 0;
		}

		int8_t Serial::FlushOutput()
		{
			if (!IsOpen())
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

			uint8_t rtn = 0;

#ifdef WIN32
			if (mFD != INVALID_HANDLE_VALUE)
			{
				rtn = PurgeComm(mFD, PURGE_TXCLEAR);
			}

			// PurgeComm returns 0 on fail, adjust to -1;
			if (rtn == 0)
			{
				rtn = -1;
			}
#elif defined __linux__
			if (mFD >= 0)
			{
				rtn = tcflush(mFD, TCOFLUSH);
			}
#endif
			if (rtn == -1)
			{
				mLastError = SerialError::FLUSH_OUTPUT_ERROR;
			}

			return 0;
		}

		int32_t Serial::Write(const void* buffer, const uint32_t size)
		{
			// TODO - implement timeout

			// Check m_status first
			if (!IsOpen())
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

			// Holds value returned from writing
			int32_t rtn = -1;

			// Write the msg over serial port that is open. 
#ifdef WIN32
			DWORD numOut = 0;
			rtn = WriteFile(mFD, buffer, size, &numOut, NULL);

			// WriteFile returns 0 on fail, adjust to -1;
			if (rtn == 0)
			{
				rtn = -1;
			}
			else
			{
				rtn = numOut;
			}
#elif defined __linux__
			rtn = write(mFD, buffer, size);
#endif

			// Check if send was successful
			if (rtn < 0)
			{
				mLastError = SerialError::WRITE_FAILURE;
				return -1;
			}

			// Return result
			return rtn;
		}

		int8_t Serial::WriteBreak(const int32_t durationInMS)
		{
			if (!IsOpen())
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

			uint8_t rtn = -1;

#ifdef WIN32
			// Sending break in windows: set, wait(Sleep), clear.
			rtn = SetCommBreak(mFD);

			if (rtn < 0)
			{
				mLastError = SerialError::WRITE_BREAK_FAILURE;
				return -1;
			}

			Sleep(durationInMS);

			rtn = ClearCommBreak(mFD);

			if (rtn < 0)
			{
				mLastError = SerialError::WRITE_BREAK_FAILURE;
				return -1;
		}
#elif defined __linux__
			rtn = tcsendbreak(mFD, durationInMS);
#endif

			if (rtn == -1)
			{
				mLastError = SerialError::WRITE_BREAK_FAILURE;
			}

			return rtn;
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
				mTitle = "Serial_" + mPort;
				return 0;
			}

			mLastError = SerialError::PORT_SET_FAILURE;
			return -1;
		}

		int8_t Serial::SetBaudrate(const BaudRate baud, const uint32_t custom )
		{
			// Catch a custom baud rate.
			if (baud == BaudRate::BAUDRATE_CUSTOM && custom != 0)
			{
				mCustomBaudRate = custom;
			}

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

		int8_t Serial::SetDelimiter(const std::string delimiter)
		{
			mDelimiter = delimiter;

			if (mDelimiter == delimiter)
			{
				return 0;
			}

			mLastError = SerialError::DELIMITER_SET_FAILURE;
			return -1;
		}

		int8_t Serial::SetBreak(const bool onoff)
		{
			if (!IsOpen())
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

			uint8_t rtn = -1;
#ifdef WIN32
			if (onoff) 
			{
				rtn = EscapeCommFunction(mFD, SETBREAK);

				if (rtn == 0)
				{
					mLastError = SerialError::WIN_SETBREAK_FAILURE;
					return -1;
				}
			}
			else 
			{
				rtn = EscapeCommFunction(mFD, CLRBREAK);

				if (rtn == 0)
				{
					mLastError = SerialError::WIN_CLRBREAK_FAILURE;
					return -1;
				}
			}
#elif defined __linux__
			if (onoff) 
			{
				rtn = ioctl(mFD, TIOCSBRK);

				if (rtn == -1)
				{
					mLastError = SerialError::LINUX_BREAK_ON_FAILURE;
					return -1;
				}
			}
			else 
			{
				rtn = ioctl(mFD, TIOCCBRK);

				if (rtn == -1)
				{
					mLastError = SerialError::LINUX_BREAK_OFF_FAILURE;
					return -1;
				}
			}
#endif
			return 0;
		}

		int8_t Serial::SetRTS(const bool onoff)
		{
			if (!IsOpen())
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

			uint8_t rtn = -1;
#ifdef WIN32
			if (onoff)
			{
				rtn = EscapeCommFunction(mFD, SETRTS);

				if (rtn == 0)
				{
					mLastError = SerialError::WIN_SETRTS_FAILURE;
					return -1;
				}
			}
			else
			{
				rtn = EscapeCommFunction(mFD, CLRRTS);

				if (rtn == 0)
				{
					mLastError = SerialError::WIN_CLRRTS_FAILURE;
					return -1;
				}
			}
#elif defined __linux__
			int8_t command = TIOCM_RTS;

			if (onoff)
			{
				rtn = ioctl(mFD, TIOCMBIS, &command);

				if (rtn == -1)
				{
					mLastError = SerialError::LINUX_RTS_ON_FAILURE;
					return -1;
				}
			}
			else
			{
				rtn = ioctl(mFD, TIOCMBIC, &command);

				if (rtn == -1)
				{
					mLastError = SerialError::LINUX_RTS_OFF_FAILURE;
					return -1;
				}
			}
#endif
			return 0;
		}

		int8_t Serial::SetDTR(const bool onoff)
		{
			if (!IsOpen())
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

			uint8_t rtn = -1;
#ifdef WIN32
			if (onoff)
			{
				rtn = EscapeCommFunction(mFD, SETDTR);

				if (rtn == 0)
				{
					mLastError = SerialError::WIN_SETDTR_FAILURE;
					return -1;
				}
			}
			else
			{
				rtn = EscapeCommFunction(mFD, CLRDTR);

				if (rtn == 0)
				{
					mLastError = SerialError::WIN_CLRDTR_FAILURE;
					return -1;
				}
			}
#elif defined __linux__
			int8_t command = TIOCM_DTR;

			if (onoff)
			{
				rtn = ioctl(mFD, TIOCMBIS, &command);

				if (rtn == -1)
				{
					mLastError = SerialError::LINUX_DTR_ON_FAILURE;
					return -1;
				}
			}
			else
			{
				rtn = ioctl(mFD, TIOCMBIC, &command);

				if (rtn == -1)
				{
					mLastError = SerialError::LINUX_DTR_OFF_FAILURE;
					return -1;
				}
			}
#endif
			return 0;
		}

		int8_t Serial::SetBinary(bool onoff)
		{
			mBinary = onoff;

			if (mBinary == onoff)
			{
				return 0;
			}

			mLastError = SerialError::BINARY_SET_FAILURE;
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

		std::string Serial::GetBaudrateAsString()
		{
			return BaudRateMap[mBaudRate];
		}

		Parity Serial::GetParity()
		{
			return mParity;
		}

		std::string Serial::GetParityAsString()
		{
			return ParityMap[mParity];
		}

		ByteSize Serial::GetByteSize()
		{
			return mByteSize;
		}

		std::string Serial::GetByteSizeAsString()
		{
			return ByteSizeMap[mByteSize];
		}

		uint32_t Serial::GetTimeout()
		{
			return mTimeout;
		}

		StopBits Serial::GetStopBits()
		{
			return mStopBits;
		}

		std::string Serial::GetStopBitsAsString()
		{
			return StopBitsMap[mStopBits];
		}

		FlowControl Serial::GetFlowControl()
		{
			return mFlowControl;
		}

		std::string Serial::GetFlowControlAsString()
		{
			return FlowControlMap[mFlowControl];
		}

		std::string Serial::GetDelimiter()
		{
			return mDelimiter;
		}

		int8_t Serial::GetCTS()
		{
			if (!IsOpen())
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

			uint8_t rtn = -1;
#ifdef WIN32
			DWORD dwModemStatus;
			
			rtn = GetCommModemStatus(mFD, &dwModemStatus);

			if (rtn == 0)
			{
				return (MS_CTS_ON & dwModemStatus) != 0;
			}
#elif defined __linux__
			int status;

			rtn = ioctl(mFD, TIOCMGET, &status);

			if (rtn >= 0)
			{
				return 0 != (status & TIOCM_CTS);
			}
#endif
			mLastError = SerialError::FAILED_GET_CTS;
			return -1;
		}

		int8_t Serial::GetDSR()
		{
			if (!IsOpen())
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

			uint8_t rtn = -1;
#ifdef WIN32
			DWORD dwModemStatus;

			rtn = GetCommModemStatus(mFD, &dwModemStatus);

			if (rtn == 0)
			{
				return (MS_DSR_ON & dwModemStatus) != 0;
			}
#elif defined __linux__
			int status;

			rtn = ioctl(mFD, TIOCMGET, &status);

			if (rtn >= 0)
			{
				return 0 != (status & TIOCM_DSR);
			}
#endif
			mLastError = SerialError::FAILED_GET_DSR;
			return -1;
		}

		int8_t Serial::GetRI()
		{
			if (!IsOpen())
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

			uint8_t rtn = -1;
#ifdef WIN32
			DWORD dwModemStatus;

			rtn = GetCommModemStatus(mFD, &dwModemStatus);

			if (rtn == 0)
			{
				return (MS_RING_ON & dwModemStatus) != 0;
			}
#elif defined __linux__
			int status;

			rtn = ioctl(mFD, TIOCMGET, &status);

			if (rtn >= 0)
			{
				return 0 != (status & TIOCM_RI);
			}
#endif
			mLastError = SerialError::FAILED_GET_RI;
			return -1;
		}

		int8_t Serial::GetCD()
		{
			if (!IsOpen())
			{
				mLastError = SerialError::SERIAL_PORT_NOT_OPEN;
				return -1;
			}

			uint8_t rtn = -1;
#ifdef WIN32
			DWORD dwModemStatus;

			rtn = GetCommModemStatus(mFD, &dwModemStatus);

			if (rtn == 0)
			{
				return (MS_RLSD_ON & dwModemStatus) != 0;
			}
#elif defined __linux__
			int status;

			rtn = ioctl(mFD, TIOCMGET, &status);

			if (rtn >= 0)
			{
				return 0 != (status & TIOCM_CD);
			}
#endif
			mLastError = SerialError::FAILED_GET_CD;
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

		int8_t Serial::SetCustomBaudrate()
		{
			return -1;
		}

		bool Serial::ValidityCheck()
		{
			if (mBaudRate == BaudRate::BAUDRATE_INVALID)
			{
				mLastError = SerialError::BAUDRATE_NOT_SET;
				return false;
			}

			if (mByteSize == ByteSize::INVALID)
			{
				mLastError = SerialError::BYTESIZE_NOT_SET;
				return false;
			}

			if (mParity == Parity::INVALID)
			{
				mLastError = SerialError::PARITY_NOT_SET;
				return false;
			}

			if (mStopBits == StopBits::INVALID)
			{
				mLastError = SerialError::STOPBITS_NOT_SET;
				return false;
			}

			return true;
		}

	} // End Namespace Communications
} // End Namespace Essentials