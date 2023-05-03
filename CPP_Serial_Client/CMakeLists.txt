# CMakeList.txt : CMake project for CPP_Serial_Client, include source and define
# project specific logic here.
#

# Add source to this project's executable.
add_executable (CPP_Serial_Client 
"main.cpp" 
 
"Source/Serial.cpp" 
"Source/Serial.h" 
"Source/Serial_Info.h")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET CPP_Serial_Client PROPERTY CXX_STANDARD_REQUIRED 20)
endif()

# TODO: Add tests and install targets if needed.
