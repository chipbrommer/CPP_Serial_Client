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
			SERIAL_WIN_OPEN_FAILURE,
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
			LINUX_BREAK_ON_FAILURE,
			LINUX_BREAK_OFF_FAILURE,
			LINUX_RTS_ON_FAILURE,
			LINUX_RTS_OFF_FAILURE,
			LINUX_DTR_ON_FAILURE,
			LINUX_DTR_OFF_FAILURE,
			BINARY_SET_FAILURE,
			WIN_SETBREAK_FAILURE,
			WIN_CLRBREAK_FAILURE,
			WIN_SETRTS_FAILURE,
			WIN_CLRRTS_FAILURE,
			WIN_SETDTR_FAILURE,
			WIN_CLRDTR_FAILURE,
			FAILED_GET_CTS,
			FAILED_GET_DSR,
			FAILED_GET_RI,
			FAILED_GET_CD,
			BAD_HANDLE,
			LINUX_BAD_HANDLE,
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
			{SerialError::SERIAL_WIN_OPEN_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::SERIAL_WIN_OPEN_FAILURE) + ": Windows serail open failure.")},
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
			{SerialError::LINUX_BREAK_ON_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::LINUX_BREAK_ON_FAILURE) + ": Linux failed to turn on break.")},
			{SerialError::LINUX_BREAK_OFF_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::LINUX_BREAK_OFF_FAILURE) + ": Linux failed to turn off break.")},
			{SerialError::LINUX_RTS_ON_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::LINUX_RTS_ON_FAILURE) + ": Linux failed to turn on RTS")},
			{SerialError::LINUX_RTS_OFF_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::LINUX_RTS_OFF_FAILURE) + ": Linux failed to turn off RTS")},
			{SerialError::LINUX_DTR_ON_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::LINUX_DTR_ON_FAILURE) + ": Linux failed to turn on DTS")},
			{SerialError::LINUX_DTR_OFF_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::LINUX_DTR_OFF_FAILURE) + ": Linux failed to turn off DTS")},
			{SerialError::BINARY_SET_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::BINARY_SET_FAILURE) + ": Failed to set binary mode.")},
			{SerialError::WIN_SETBREAK_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::WIN_SETBREAK_FAILURE) + ": WIN32 failed to set break.")},
			{SerialError::WIN_CLRBREAK_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::WIN_CLRBREAK_FAILURE) + ": WIN32 failed to clear break.")},
			{SerialError::WIN_SETRTS_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::WIN_SETRTS_FAILURE) + ": WIN32 failed to set RTS")},
			{SerialError::WIN_CLRRTS_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::WIN_CLRRTS_FAILURE) + ": WIN32 failed to clear RTS")},
			{SerialError::WIN_SETDTR_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::WIN_SETDTR_FAILURE) + ": WIN32 failed to set DTR")},
			{SerialError::WIN_CLRDTR_FAILURE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::FAILED_GET_CTS) + ": WIN32 failed to clear DTR")},
			{SerialError::FAILED_GET_CTS,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::FAILED_GET_CTS) + ": Failed to get CTS bit")},
			{SerialError::FAILED_GET_DSR,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::FAILED_GET_DSR) + ": Failed to get DSR bit")},
			{SerialError::FAILED_GET_RI,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::FAILED_GET_RI) + ": Failed to get RI bit")},
			{SerialError::FAILED_GET_CD,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::FAILED_GET_CD) + ": Failed to get CD bit")},
			{ SerialError::BAD_HANDLE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::BAD_HANDLE) + ": Bad file handle.") },
			{ SerialError::LINUX_BAD_HANDLE,
			std::string("Error Code " + std::to_string((uint8_t)SerialError::LINUX_BAD_HANDLE) + ": Bad file handle.") },
		};

		/// <summary>Enum for typical baudrates</summary>
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

		/// <summary>Baud Rate enum to string map</summary>
		static std::map<BaudRate, std::string> BaudRateMap
		{
			{BaudRate::BAUDRATE_50,			std::string("Baud Rate 50.")},
			{BaudRate::BAUDRATE_75,			std::string("Baud Rate 75.")},
			{BaudRate::BAUDRATE_110,		std::string("Baud Rate 110")},
			{BaudRate::BAUDRATE_134,		std::string("Baud Rate 134")},
			{BaudRate::BAUDRATE_150,		std::string("Baud Rate 150")},
			{BaudRate::BAUDRATE_200,		std::string("Baud Rate 200.")},
			{BaudRate::BAUDRATE_300,		std::string("Baud Rate 300.")},
			{BaudRate::BAUDRATE_600,		std::string("Baud Rate 600.")},
			{BaudRate::BAUDRATE_1200,		std::string("Baud Rate 1200")},
			{BaudRate::BAUDRATE_1800,		std::string("Baud Rate 1800")},
			{BaudRate::BAUDRATE_2400,		std::string("Baud Rate 2400")},
			{BaudRate::BAUDRATE_4800,		std::string("Baud Rate 4800.")},
			{BaudRate::BAUDRATE_9600,		std::string("Baud Rate 9600.")},
			{BaudRate::BAUDRATE_14400,		std::string("Baud Rate 14400.")},
			{BaudRate::BAUDRATE_19200,		std::string("Baud Rate 19200")},
			{BaudRate::BAUDRATE_38400,		std::string("Baud Rate 38400")},
			{BaudRate::BAUDRATE_57600,		std::string("Baud Rate 57600")},
			{BaudRate::BAUDRATE_76800,		std::string("Baud Rate 76800.")},
			{BaudRate::BAUDRATE_115200,		std::string("Baud Rate 115200.")},
			{BaudRate::BAUDRATE_128000,		std::string("Baud Rate 128000.")},
			{BaudRate::BAUDRATE_256000,		std::string("Baud Rate 256000")},
			{BaudRate::BAUDRATE_460800,		std::string("Baud Rate 460800")},
			{BaudRate::BAUDRATE_921600,		std::string("Baud Rate 921600")},
			{BaudRate::BAUDRATE_CUSTOM,		std::string("Baud Rate custom")},
			{BaudRate::BAUDRATE_INVALID,	std::string("Baud Rate Invalid.")},
		};

		/// <summary>enum for the byte size options for the port.</summary>
		enum class ByteSize : uint8_t
		{
			FIVE,
			SIX,
			SEVEN,
			EIGHT,
			INVALID,
		};

		/// <summary>Byte Size enum to string map</summary>
		static std::map<ByteSize, std::string> ByteSizeMap
		{
			{ByteSize::FIVE,	std::string("Bite Size 5.")},
			{ByteSize::SIX,		std::string("Bite Size 6.")},
			{ByteSize::SEVEN,	std::string("Bite Size 7.")},
			{ByteSize::EIGHT,	std::string("Bite Size 8.")},
			{ByteSize::INVALID,	std::string("Bite Size invalid.")},
		};

		/// <summary>enum for the parity options for the port.</summary>
		enum class Parity : uint8_t
		{
			NONE,
			ODD,
			EVEN,
			MARK,
			SPACE,
			INVALID,
		};

		/// <summary>Parity enum to string map</summary>
		static std::map<Parity, std::string> ParityMap
		{
			{Parity::NONE,		std::string("Parity None.")},
			{Parity::ODD,		std::string("Parity Odd.")},
			{Parity::EVEN,		std::string("Parity Even")},
			{Parity::MARK,		std::string("Parity Mark")},
			{Parity::SPACE,		std::string("Parity Space")},
			{Parity::INVALID,	std::string("Parity invalid.")},
		};

		/// <summary>enum for the stop bits options for the port.</summary>
		enum class StopBits : uint8_t
		{
			ONE,
			TWO,
			ONE_FIVE,
			INVALID
		};

		/// <summary>Stop Bits enum to string map</summary>
		static std::map<StopBits, std::string> StopBitsMap
		{
			{StopBits::ONE,			std::string("StopBits One.")},
			{StopBits::TWO,			std::string("StopBits Two.")},
			{StopBits::ONE_FIVE,	std::string("StopBits One point five")},
			{StopBits::INVALID,		std::string("StopBits Invalid")},
		};

		/// <summary>enum for the flow control options for the port.</summary>
		enum class FlowControl : uint8_t
		{
			NONE,
			SOFTWARE,
			HARDWARE,
		};

		static std::map<FlowControl, std::string> FlowControlMap
		{
			{FlowControl::NONE,		std::string("Flow Control None.")},
			{FlowControl::SOFTWARE,	std::string("Flow Control Software.")},
			{FlowControl::HARDWARE,	std::string("Flow Control Hardware.")},
		};

		/// <summary>A class for communication over a serial port. </summary>
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

			/// <summary> Reconfigure a port after it has been opened.</summary>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t ReconfigurePort();

			/// <summary>Check if port is open.</summary>
			/// <returns>True if open, false if closed.</returns>
			bool IsOpen();

			/// <summary>Read from serial into the passed in buffer.</summary>
			/// <param name="buffer"> -[out]- Pointer to a buffer to read into</param>
			/// <param name="size"> -[in]- Desired size to be read</param>
			/// <returns> 0+ if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
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
			
			/// <summary>Writes a break over the serial port</summary>
			/// <param name="durationInMS"> -[in]- Duration in MS to write the break.</param>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
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

			/// <summary>Set the port to break until called to stop</summary>
			/// <param name="onoff"> -[in]- Flag to turn break on or off.</param>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t SetBreak(const bool onoff);

			/// <summary>Enable or Disable the RTS (request-to-send) signal.</summary>
			/// <param name="onoff"> -[in]- Flag to turn RTS on or off.</param>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t SetRTS(const bool onoff);

			/// <summary>Enable or Disable the DTR (data-terminal-ready) signal.</summary>
			/// <param name="onoff"> -[in]- Flag to turn DTR on or off.</param>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t SetDTR(const bool onoff);

			/// <summary>Set the serial port for binary</summary>
			/// <param name="onoff"> -[in]- Flag to turn bindary on or off.</param>
			/// <returns>0 if successful, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int8_t SetBinary(bool onoff);

			/// <summary>Get the current port in string format</summary>
			/// <returns>The set port</returns>
			std::string GetPort();

			/// <summary>Get the current BaudRate set for the port</summary>
			/// <returns>The current baud rate</returns>
			BaudRate GetBaudrate();

			/// <summary>Get the current BaudRate set for the port as a string.</summary>
			/// <returns>The current baud rate in string format</returns>
			std::string GetBaudrateAsString();

			/// <summary>Get the current Parity set for the port</summary>
			/// <returns>The current parity</returns>
			Parity GetParity();

			/// <summary>Get the current Parity set for the port as a string.</summary>
			/// <returns>The current parity in string format</returns>
			std::string GetParityAsString();

			/// <summary>Get the current ByteSize set for the port</summary>
			/// <returns>The current byte size</returns>
			ByteSize GetByteSize();

			/// <summary>Get the current Byte Size set for the port as a string.</summary>
			/// <returns>The current byte size in string format</returns>
			std::string GetByteSizeAsString();

			/// <summary>Get the current timeout set for the port</summary>
			/// <returns>The current timeout, 0 = not set</returns>
			uint32_t GetTimeout();

			/// <summary>Get the current StopBits set for the port</summary>
			/// <returns>The current StopBits</returns>
			StopBits GetStopBits();

			/// <summary>Get the current StopBits set for the port as a string.</summary>
			/// <returns>The current stop bits in string format</returns>
			std::string GetStopBitsAsString();

			/// <summary>Get the current Flow Controller set for the port</summary>
			/// <returns>The current Flow Control setting</returns>
			FlowControl GetFlowControl();

			/// <summary>Get the current Flow Control set for the port as a string.</summary>
			/// <returns>The current flow control in string format</returns>
			std::string GetFlowControlAsString();

			/// <summary>Get the current delimiter set for the port</summary>
			/// <returns>The current delimiter, "\n" = not set</returns>
			std::string GetDelimiter();

			/// <summary>Check if the CTS (clear-to-send) signal is on.</summary>
			/// <return>1 if on, 0 if off. -1 on failure, Call Serial::GetLastError to find out more</return>
			int8_t GetCTS();

			/// <summary>Check if the DSR (data-set-ready) signal is on.</summary>
			/// <return>1 if on, 0 if off. -1 on failure, Call Serial::GetLastError to find out more.</return>
			int8_t GetDSR();

			/// <summary>Check if the ring indicator signal is on.</summary>
			/// <return>1 if on, 0 if off. -1 on failure, Call Serial::GetLastError to find out more.</return>
			int8_t GetRI();

			/// <summary>Check if the RLSD (receive-line-signal-detect) signal is on.</summary>
			/// <return>1 if on, 0 if off. -1 on failure, Call Serial::GetLastError to find out more.</return>
			int8_t GetCD();

			/// <summary>Check if the port is in binary mode.</summary>
			/// <returns>True if in binary mode, false if not.</returns>
			bool GetBinary();

			/// <summary>Get the number of bytes available on the serial port to be read.</summary>
			/// <returns>0+ on success indicating number of available bytes, -1 if fails. Call Serial::GetLastError to find out more.</returns>
			int32_t GetInQueueLength();

			/// <summary>Get the last error in string format</summary>
			/// <returns>The last error in a formatted string</returns>
			std::string GetLastError();

		protected:
		private:

			/// <summary>Check if the Baudrate, Parity, ByteSize, and StopBits are valid.</summary>
			/// <returns>True if valid, false if not.</returns>
			bool ValidityCheck();

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