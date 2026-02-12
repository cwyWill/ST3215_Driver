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
    std::this_thread::sleep_for(std::chrono::microseconds(5));

    // enable motors
    // move to position 1000 and 2000
    if ( !act.enableTorque() ) {
        std::cerr << "Fail to enable torque.\n";
    }
    std::this_thread::sleep_for(std::chrono::microseconds(5));
    if ( !act.setPosition( std::vector<int>{0, 1}, std::vector<int>{400, 2000} ) ) {
        std::cerr << "Fail to set position.\n";
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    act.updateAllFeedback();
    
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "Motor 0 position: " << act.getPosition({0})[0] << ", Motor 1 position: " << act.getPosition({1})[0] << '\n';

    if ( !act.setPosition( std::vector<int>{0, 1}, std::vector<int>{2000, 500 } ) ) {
        std::cerr << "Fail to set position.\n";
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    act.updateAllFeedback();
    std::cout << "Motor 0 position: " << act.getPosition({0})[0] << ", Motor 1 position: " << act.getPosition({1})[0] << '\n';
    
    return 0;
}
