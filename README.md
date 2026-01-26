# ST3215 Driver
The C++ driver of ST3215 from Waveshare. The code is modified from the original scripts provided by Waveshare with the serial replaced with boost::asio serial realization.

## Dependencies
+ boost::system
+ Written with consideration of C++ 17

## Build
``` bash
# in the working directory
cmake -B build
cmake --build build
```

## Current Progress
Capable of writing and reading from the EEPROM.

## TODO
+ Append more write and read functions.
+ Add commments.
+ Rename functions' and variables' names for better readability.
+ Multiple ID operation.
+ Handle read timeout.
+ Serial buffer flush.
