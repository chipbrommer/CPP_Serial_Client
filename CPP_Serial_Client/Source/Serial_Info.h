#include <map>
//#include <format>

namespace Essentials
{
	namespace Communications
	{
		constexpr static uint8_t SERIAL_VERSION_MAJOR = 0;
		constexpr static uint8_t SERIAL_VERSION_MINOR = 1;
		constexpr static uint8_t SERIAL_VERSION_PATCH = 0;
		constexpr static uint8_t SERIAL_VERSION_BUILD = 0;

		//static std::string SerialVersion = std::format("Serial Client v{}.{}.{} - b{}",
		//	SERIAL_VERSION_MAJOR, SERIAL_VERSION_MINOR, 
		//	SERIAL_VERSION_PATCH, SERIAL_VERSION_BUILD);

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
			FLUSH_ERROR,
			HANDLE_SETUP_FAILURE,
			SET_COMMSTATE_FAILURE,
			SET_COMMTIMEOUT_FAILURE,
			QUEUE_LENGTH_READ_FAILURE,
			SERIAL_LINUX_OPEN_FAILURE,
			SERIAL_LINUX_GETATTRIBUTES_FAILURE,
			SERIAL_LINUX_SETATTRIBUTES_FAILURE,
		};

		/// <summary>A Map to convert an error value to a readable string.</summary>
		static std::map<SerialError, std::string> SerialErrorMap
		{
			//{SerialError::NONE,	std::format("Error Code {} - No error.\n",(uint8_t)SerialError::NONE)},
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