// #include "SMS_STS.h"
// #include <boost/asio.hpp>
#include "Actuator_Handler.h"
#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

// template class Actuator_Handler<2>;

int main() {

    // SMS_STS st{"/dev/ttyACM0", BaudRate::r_1M};
    // std::cout << "Port starts successfully\n";

    Actuator_Handler<2> act {
        std::array<ServoMotorState, 2> {
            ServoMotorState{ .ID = 4 },
            ServoMotorState{ .ID = 5 }
        },
        "/dev/ttyACM0",
        BaudRate::r_1M
    };

    if ( act.selfInsepct() ) {
        std::cout << "All motors are responding.\n";
    } else {
        std::cerr << "Some motors are not responding.\n";
    }

    // enable motors
    // move to position 1000 and 2000
    if ( !act.enableTorque() ) {
        std::cerr << "Fail to enable torque.\n";
    }
    if ( !act.setPosition( std::vector<int>{0, 1}, std::vector<int>{1000, 2000} ) ) {
        std::cerr << "Fail to set position.\n";
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    if ( !act.setPosition( std::vector<int>{0, 1}, std::vector<int>{2000, 1000} ) ) {
        std::cerr << "Fail to set position.\n";
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    return 0;
}
