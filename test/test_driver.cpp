#include "SMS_STS.h"
// #include <boost/asio.hpp>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

int main() {
    using namespace std::string_literals;

    SMS_STS st{"/dev/ttyACM0", BaudRate::r_1M};
    std::cout << "Port starts successfully\n";

    st.enableTorque(1);
    st.writePosition(1, 2048);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Motor load: " << st.readLoad(1) << '\n';

    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Motor status: " << std::boolalpha << st.isMoving(1)  << ", position: "<< st.readPosition(1) << '\n';

    st.writePosition(1, 4096);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "Motor status: " << std::boolalpha << st.isMoving(1)  << ", position: "<< st.readPosition(1) << '\n';
    st.disableTorque(1);
    
    return 0;
}
