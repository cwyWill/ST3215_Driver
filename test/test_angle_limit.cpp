#include "SMS_STS.h"
// #include <boost/asio.hpp>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

inline void sleep_sec(int sec) {
    std::this_thread::sleep_for(std::chrono::seconds(sec));
}


int main() {
    using namespace std::string_literals;

    SMS_STS st{"/dev/ttyACM0", BaudRate::r_1M};
    std::cout << "Port starts successfully\n";


    constexpr int ID {4};

    st.printInfo(ID);

    std::cout << "Set motor position to " << 2048 << '\n';
    st.writePosition(ID, 2048);
    
    std::cout << "Motor status: " << std::boolalpha << st.isMoving(ID)  << ", position: "<< st.readPosition(ID) << '\n';

    sleep_sec(1);
    st.printInfo(ID);
    std::cout << "Motor status: " << std::boolalpha << st.isMoving(ID)  << ", position: "<< st.readPosition(ID) << '\n';

    sleep_sec(8);
    st.printInfo(ID);
    std::cout << "Motor status: " << std::boolalpha << st.isMoving(ID)  << ", position: "<< st.readPosition(ID) << '\n';

    std::cout << "Set motor position to " << 2048+4096*2 << '\n';
    st.writePosition(ID, 4095);
    sleep_sec(8);
    st.printInfo(ID);
    std::cout << "Motor status: " << std::boolalpha << st.isMoving(ID)  << ", position: "<< st.readPosition(ID) << '\n';
    
    return 0;
}
