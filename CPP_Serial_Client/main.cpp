// CPP_Serial_Client.cpp : Defines the entry point for the application.
//

#include "Source/Serial.h"
#include <iostream>

int main()
{
	std::cout << "Hello CMake." << std::endl;
	std::cout << Essentials::Communications::SerialVersion;

	Essentials::Communications::Serial serial;
	serial.Configure("/dev/ttyS0", 
		Essentials::Communications::BaudRate::BAUDRATE_115200, 
		Essentials::Communications::ByteSize::EIGHT, 
		Essentials::Communications::Parity::NONE);
	serial.SetDelimiter("&");
	serial.Open();

	std::string tmp = "Hello World";
	serial.Write(tmp.c_str(), tmp.length());

	char buffer[250] = { 0 };
	int i = serial.Read(buffer, sizeof(buffer));

	std::cout << "Read: " << i << "\n";

	std::string c;
	c = serial.GetLastError();
	std::cout << c << std::endl;

	return 0;

}
