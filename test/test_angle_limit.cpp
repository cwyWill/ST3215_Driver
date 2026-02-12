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


    std::cout << "Set motor position to " << 2048 << '\n';
    st.writePosition(ID, 2048);
    
    // std::cout << "Motor status: " << std::boolalpha << st.isMoving(ID).value  << ", position: "<< st.readPosition(ID).value << '\n';

    sleep_sec(1);
    // std::cout << "Motor status: " << std::boolalpha << st.isMoving(ID).value  << ", position: "<< st.readPosition(ID).value << '\n';

    sleep_sec(1);
    // std::cout << "Motor status: " << std::boolalpha << st.isMoving(ID).value  << ", position: "<< st.readPosition(ID).value << '\n';

    std::cout << "Set motor position to " << 4095 << '\n';
    st.writePosition(ID, 4095);
    sleep_sec(1);
    // std::cout << "Motor status: " << std::boolalpha << st.isMoving(ID).value  << ", position: "<< st.readPosition(ID).value << '\n';
    
    return 0;
}
