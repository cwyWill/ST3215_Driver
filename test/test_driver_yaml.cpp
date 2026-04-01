#include "Actuator_Handler.h"
#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include "joint_loader.h"

int main() {

    SMS_STS st{"/dev/ttyACM0", BaudRate::r_1M};

    using namespace Actuator;
    Actuator_Handler<1> act {
        loadMotorsFromYAML<1>("joint_config.yaml", st),
        st
    };

    // self inspection
    if ( act.selfInspect() ) {
        std::cout << "All motors are responding.\n";
    } else {
        std::cerr << "Some motors are not responding.\n";
    }
    std::this_thread::sleep_for(std::chrono::microseconds(5));

    act.setAngle( std::array<double, 1>{ 0 } );
    std::this_thread::sleep_for(std::chrono::seconds(2));

    act.updateAllFeedback();
    std::cout << "Motor 0 position: " << act.getAngle({0})[0] << '\n';


    act.setAngle( std::array<double, 1>{ M_PI_2 } );
    std::this_thread::sleep_for(std::chrono::seconds(2));

    act.updateAllFeedback();
    std::cout << "Motor 0 position: " << act.getAngle({0})[0] << '\n';
    
    std::this_thread::sleep_for(std::chrono::seconds(2));

    act.setAngle( std::array<double, 1>{ 0 } );
    std::this_thread::sleep_for(std::chrono::seconds(2));
    act.updateAllFeedback();
    std::cout << "Motor 0 position: " << act.getAngle({0})[0] << '\n';


    return 0;
}
