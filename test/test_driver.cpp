#include "SMS_STS.h"
// #include <boost/asio.hpp>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    SMS_STS st{"/dev/ttyACM0", BaudRate::r_1M};

    std::cout << "Port starts successfully\n";
    st.enableTorque(1);
    st.writePosEx(1, 2048, 0, 20);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Motor load: " << st.readLoad(1) << '\n';
    std::this_thread::sleep_for(std::chrono::seconds(1));
    int pos {st.readPosition(1)};
    std::cout << "Motor position: " << pos << '\n';
    st.writePosEx(1, 1024, 0, 20);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "Motor position: " << st.readPosition(1) << '\n';
    st.disableTorque(1);
    
    return 0;
}
