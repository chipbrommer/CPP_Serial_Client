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
#include <windows.h>					// Windows basic include. File APIs
#elif defined __linux__
#include <termios.h>					
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/types.h>
#endif
#include <map>							// Error enum to strings.
#include <string>						// Strings
#include <cstring>						// Memcpy
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
		constexpr static uint8_t SERIAL_VERSION_MAJOR = 0;
		constexpr static uint8_t SERIAL_VERSION_MINOR = 1;
		constexpr static uint8_t SERIAL_VERSION_PATCH = 0;
		constexpr static uint8_t SERIAL_VERSION_BUILD = 0;

		static std::string SerialVersion = "Serial Client v" +
			std::to_string((uint8_t)SERIAL_VERSION_MAJOR) + "." +
			std::to_string((uint8_t)SERIAL_VERSION_MINOR) + "." +
			std::to_string((uint8_t)SERIAL_VERSION_PATCH) + " - b" +
			std::to_string((uint8_t)SERIAL_VERSION_BUILD) + ".\n";

		enum class SerialError : uint8_t
		{
			NONE,
			PORT_NOT_SET,
			PORT_SET_FAILURE,
			BAUDRATE_NOT_SET,
			BAUDRATE_SET_FAILURE,
			PARITY_NOT_SET,
			PARITY_SET_FAILURE,
			BYTESIZE_NOT_SET,
			BYTESIZE_SET_FAILURE,
			TIMEOUT_SET_FAILURE,
			STOPBITS_NOT_SET,
			STOPBITS_SET_FAILURE,
			FLOWCONTROL_SET_FAILURE,
			DELIMITER_SET_FAILURE,
			SERIAL_PORT_NOT_OPEN,
			SERIAL_PORT_ALREADY_OPEN,
			CLOSE_FAILURE,
			FLUSH_INPUT_ERROR,
			FLUSH_OUTPUT_ERROR,
			FLUSH_IO_ERROR,
			HANDLE_SETUP_FAILURE,
			SET_COMMSTATE_FAILURE,
			SET_COMMTIMEOUT_FAILURE,
			QUEUE_LENGTH_READ_FAILURE,
			SERIAL_LINUX_OPEN_FAILURE,
			SERIAL_LINUX_GETATTRIBUTES_FAILURE,
			SERIAL_LINUX_SETATTRIBUTES_FAILURE,
			WRITE_FAILURE,
			WRITE_LINE_FAILURE,
			WRITE_BREAK_FAILURE,
			READ_FAILURE,
			READ_LINE_FAILURE,
			WIN32_WAIT_READBALE,
		};

		/// <summary>A Map to convert an error value to a readable string.</summary>
		static std::map<SerialError, std::string> SerialErrorMap
		{
			{SerialError::NONE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::NONE) + ": No error.")},
			{SerialError::PORT_NOT_SET,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::PORT_NOT_SET) + ": Port not set.")},
			{SerialError::PORT_SET_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::PORT_SET_FAILURE) + ": Failed to set port.")},
			{SerialError::BAUDRATE_NOT_SET,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::BAUDRATE_NOT_SET) + ": Baudrate not set.")},
			{SerialError::BAUDRATE_SET_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::BAUDRATE_SET_FAILURE) + ": Baudrate set failure.")},
			{SerialError::PARITY_NOT_SET,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::PARITY_NOT_SET) + ": Parity not set.")},
			{SerialError::PARITY_SET_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::PARITY_SET_FAILURE) + ": Parity set failure.")},
			{SerialError::BYTESIZE_NOT_SET,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::BYTESIZE_NOT_SET) + ": Bytesize not set.")},
			{SerialError::BYTESIZE_SET_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::BYTESIZE_SET_FAILURE) + ": Bytesize set failure.")},
			{SerialError::TIMEOUT_SET_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::TIMEOUT_SET_FAILURE) + ": Timeout set failure.")},
			{SerialError::STOPBITS_NOT_SET,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::STOPBITS_NOT_SET) + ": Stopbits not set.")},
			{SerialError::STOPBITS_SET_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::STOPBITS_SET_FAILURE) + ": Stopbits set failure.")},
			{SerialError::FLOWCONTROL_SET_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::FLOWCONTROL_SET_FAILURE) + ": Flow control set failure.")},
			{SerialError::DELIMITER_SET_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::DELIMITER_SET_FAILURE) + ": Delimiter set failure.")},
			{SerialError::SERIAL_PORT_NOT_OPEN,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::SERIAL_PORT_NOT_OPEN) + ": Serial port not open.")},
			{SerialError::SERIAL_PORT_ALREADY_OPEN,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::SERIAL_PORT_ALREADY_OPEN) + ": Serial port already open.")},
			{SerialError::CLOSE_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::CLOSE_FAILURE) + ": Close failure.")},
			{SerialError::FLUSH_INPUT_ERROR,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::FLUSH_INPUT_ERROR) + ": Flush input error.")},
			{SerialError::FLUSH_OUTPUT_ERROR,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::FLUSH_OUTPUT_ERROR) + ": Flush output error.")},
			{SerialError::FLUSH_IO_ERROR,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::FLUSH_IO_ERROR) + ": Flush input/output error.")},
			{SerialError::HANDLE_SETUP_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::HANDLE_SETUP_FAILURE) + ": Handle setup failure")},
			{SerialError::SET_COMMSTATE_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::SET_COMMSTATE_FAILURE) + ": Set COMMSTATE failure.")},
			{SerialError::SET_COMMTIMEOUT_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::SET_COMMTIMEOUT_FAILURE) + ": Set COMMTIMEOUT failure.")},
			{SerialError::QUEUE_LENGTH_READ_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::QUEUE_LENGTH_READ_FAILURE) + ": Queue length read failure.")},
			{SerialError::SERIAL_LINUX_OPEN_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::SERIAL_LINUX_OPEN_FAILURE) + ": Linux serial open failure.")},
			{SerialError::SERIAL_LINUX_GETATTRIBUTES_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::SERIAL_LINUX_GETATTRIBUTES_FAILURE) + ": Linux serial GETATTRIBUTES failure.")},
			{SerialError::SERIAL_LINUX_SETATTRIBUTES_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::SERIAL_LINUX_SETATTRIBUTES_FAILURE) + ": Linux serial SETATTRIBUTES failure.")},
			{SerialError::WRITE_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::WRITE_FAILURE) + ": Write failure.")},
			{SerialError::WRITE_LINE_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::WRITE_LINE_FAILURE) + ": Write line failure.")},
			{SerialError::WRITE_BREAK_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::WRITE_BREAK_FAILURE) + ": Write break failure.")},
			{SerialError::READ_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::READ_FAILURE) + ": Read failure.")},
			{SerialError::READ_LINE_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::READ_LINE_FAILURE) + ": Read line failure.")},
			{SerialError::WIN32_WAIT_READBALE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::WIN32_WAIT_READBALE) + ": WIN32 wait readable")},
		};

		enum class BaudRate : uint32_t
		{
			BAUDRATE_50,		// Posix only.
			BAUDRATE_75,		// Posix only.
			BAUDRATE_110,
			BAUDRATE_134,		// Posix only.
			BAUDRATE_150,		// Posix only.
			BAUDRATE_200,		// Posix only.
			BAUDRATE_300,
			BAUDRATE_600,
			BAUDRATE_1200,
			BAUDRATE_1800,		// Posix only.
			BAUDRATE_2400,
			BAUDRATE_4800,
			BAUDRATE_9600,
			BAUDRATE_14400,		// Windows only. 
			BAUDRATE_19200,
			BAUDRATE_38400,
			BAUDRATE_57600,
			BAUDRATE_76800,		// Posix only.
			BAUDRATE_115200,
			BAUDRATE_128000,	// Windows only.
			BAUDRATE_256000,	// Windows only.
			BAUDRATE_460800,
			BAUDRATE_921600,
			BAUDRATE_CUSTOM,
			BAUDRATE_INVALID,
		};

		enum class ByteSize : uint8_t
		{
			FIVE,
			SIX,
			SEVEN,
			EIGHT,
			INVALID,
		};

		enum class Parity : uint8_t
		{
			NONE,
			ODD,
			EVEN,
			MARK,
			SPACE,
			INVALID,
		};

		enum class StopBits : uint8_t
		{
			ONE,
			TWO,
			ONE_FIVE,
			INVALID
		};

		enum class FlowControl : uint8_t
		{
			NONE,
			SOFTWARE,
			HARDWARE,
		};

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
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t Configure(const std::string port, const BaudRate baud, const ByteSize bytes, const Parity parity);

			/// <summary>Opens a serial connection.</summary>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t Open();

			/// <summary>Check if port is open.</summary>
			/// <returns>True if open, false if closed.</returns>
			bool IsOpen();

			/// <summary></summary>
			/// <returns></returns>
			int8_t WaitReadable();

			/// <summary>Read from serial into the passed in buffer.</summary>
			/// <param name="buffer"> -[out]- Pointer to a buffer to read into</param>
			/// <param name="size"> -[in]- Desired size to be read</param>
			/// <returns>0+ if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int32_t Read(void* buffer, const uint32_t size);

			/// <summary>Read serial one byte at a time until end line is git or delimiter is caught</summary>
			/// <param name="buffer"> -[out]- Pointer to a buffer to store read contents.</param>
			/// <param name="size"> -[in]- Max size allowed to read.</param>
			/// <param name="delimiter"> -[opt/in]- Delimiter to be used for this read. Defaults to "\n" or the set delimiter</param>
			/// <returns>0+ if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int32_t ReadLine(void* buffer, const uint32_t size, std::string delimiter = "\n");

			/// <summary>Flush the input and output of a serial port</summary>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t Flush();

			/// <summary>Flush the input of a serial port</summary>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t FlushInput();

			/// <summary>Flush the output of a serial port</summary>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t FlushOutput();

			/// <summary>Writes a buffer of specified size over a serial port</summary>
			/// <param name="buffer"> -[in]- Pointer to a buffer to be sent</param>
			/// <param name="size"> -[in]- Size of the data to be sent</param>
			/// <returns>0+ if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int32_t Write(const void* buffer, const uint32_t size);


			int8_t WriteBreak(const int32_t durationInMS);

			/// <summary>Closes a serial connection</summary>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t Close();

			/// <summary>Sets the port to connect on</summary>
			/// <param name="port"> -[in]- Port to connect on.</param>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t SetSerialPort(const std::string port);

			/// <summary>Sets the baudrate for the serial port</summary>
			/// <param name="baud"> -[in]- Baudrate for the port.</param>
			/// <param name="custom"> -[opt/in]- Specified Baudrate when using BaudRate::BAUDRATE_CUSTOM</param>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t SetBaudrate(const BaudRate baud, const uint32_t custom = 0);

			/// <summary>Sets the parity for the serial port</summary>
			/// <param name="parity"> -[in]- Parity for the port.</param>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t SetParity(const Parity parity);

			/// <summary>Sets the bytesize for the serial port</summary>
			/// <param name="size"> -[in]- BytesSize for the port.</param>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t SetByteSize(const ByteSize size);

			/// <summary>Sets the timeout for the serial port</summary>
			/// <param name="timeoutMS"> -[in]- Timeout for the serial port in MSecs</param>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t SetTimeout(const uint32_t timeoutMS);

			/// <summary>Sets the stopbits for the serial port</summary>
			/// <param name="bits"> -[in]- StopBits for the port.</param>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t SetStopBits(const StopBits bits);

			/// <summary>Sets the flow controller for the serial port</summary>
			/// <param name="flow"> -[in]- Desired flow control designation for the port.</param>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t SetFlowControl(const FlowControl flow);

			/// <summary>Set the delimiter for reading serial one byte at a time.</summary>
			/// <param name="delimiter"> -[in]- Delimiter to be used when parsing the serial read one byte at a time.</param>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t SetDelimiter(const std::string delimiter);

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
			/// <returns>The current delimiter, "\n" = not set</returns>
			std::string GetDelimiter();


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

			/// <summary>Sets a custom baud rate not automatically supported by specific OS.</summary>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
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
			std::string		mDelimiter;			// Delimiter for the serial stream

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