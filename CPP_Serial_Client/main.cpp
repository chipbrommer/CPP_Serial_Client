// CPP_Serial_Client.cpp : Defines the entry point for the application.
//

#include "Source/Serial.h"
#include <iostream>

int main()
{
	std::cout << "Hello CMake." << std::endl;
	std::cout << Essentials::Communications::SerialVersion;

	Essentials::Communications::Serial serial;
	serial.Configure("COMM4", Essentials::Communications::BaudRate::BAUDRATE_115200, Essentials::Communications::ByteSize::EIGHT, Essentials::Communications::Parity::NONE);
	serial.SetDelimiter("&");

	std::string c;
	c = serial.GetLastError();
	std::cout << c << std::endl;

	int8_t buffer[200] = { 0 };
	serial.ReadLine(buffer, sizeof(buffer));

	return 0;

}
