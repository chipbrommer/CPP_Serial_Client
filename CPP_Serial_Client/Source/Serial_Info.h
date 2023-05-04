///////////////////////////////////////////////////////////////////////////////
//!
//! @file		Serial_Info.cpp
//! 
//! @brief		Serial class information related enums and data. 
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
#include	<map>						// Map for error enum to string. 
#include	<string>					// Strings
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

		static std::string SerialVersion = "Serial Client v"		+
			std::to_string((uint8_t)SERIAL_VERSION_MAJOR) + "."		+
			std::to_string((uint8_t)SERIAL_VERSION_MINOR) + "."		+
			std::to_string((uint8_t)SERIAL_VERSION_PATCH) + " - b"	+
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
	} // End Namespace Communications
} // End Namespace Essentials