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
    st.writePosEx(1, 2048, 0, 0);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    int pos {st.ReadPos(1)};
    std::cout << "Motor position: " << pos << '\n';
    st.writePosEx(1, 1024, 0, 0);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    st.disableTorque(1);
    
    return 0;
}
