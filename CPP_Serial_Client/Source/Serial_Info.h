#include <map>
#include <format>

namespace Essentials
{
	namespace Communications
	{
		constexpr static uint8_t SERIAL_VERSION_MAJOR = 0;
		constexpr static uint8_t SERIAL_VERSION_MINOR = 1;
		constexpr static uint8_t SERIAL_VERSION_PATCH = 0;
		constexpr static uint8_t SERIAL_VERSION_BUILD = 0;

		static std::string SerialVersion = std::format("Serial Client v{}.{}.{} - b{}",
			SERIAL_VERSION_MAJOR, SERIAL_VERSION_MINOR, 
			SERIAL_VERSION_PATCH, SERIAL_VERSION_BUILD);

		enum class SerialError : uint8_t
		{
			NONE,
		};

		/// <summary>A Map to convert an error value to a readable string.</summary>
		static std::map<SerialError, std::string> SerialErrorMap
		{
			{SerialError::NONE,	std::format("Error Code {} - No error.\n",(uint8_t)SerialError::NONE)},
		};

		enum class BaudRate : uint8_t
		{
			BAUD_110,
			BAUD_300,
			BAUD_600,
			BAUD_1200,
			BAUD_2400,
			BAUD_4800,
			BAUD_9600,
			BAUD_14400,
			BAUD_19200,
			BAUD_38400,
			BAUD_57600,
			BAUD_115200,
			BAUD_128000,
			BAUD_256000,
			BAUD_INVALID,
		};

		enum class ByteSize : uint8_t
		{
			FIVE = 5,
			SIX = 6,
			SEVEN = 7,
			EIGHT = 8,
		};

		enum class Parity : uint8_t
		{
			NONE,
			ODD,
			EVEN,
			MARK,
			SPACE,
		};

		enum class StopBits : uint8_t
		{
			ONE,
			TWO,
			ONE_FIVE,
		};

		enum class FlowControl : uint8_t
		{
			NONE,
			SOFTWARE,
			HARDWARE,
		};
	} // End Namespace Communications
} // End Namespace Essentials