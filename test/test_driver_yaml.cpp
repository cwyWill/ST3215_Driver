#include "Actuator_Handler.h"
#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <yaml_file>\n";
        return 1;
    }

    SMS_STS st{"/dev/ttyACM0", BaudRate::r_1M};

    using namespace Actuator;
    Actuator_Handler<2> act {
        loadMotorsFromYAML<2>(argv[1]),
        st
    };
    // std::cout << act << '\n';
    act.print();

    // self inspection
    if ( act.selfInspect() ) {
        std::cout << "All motors are responding.\n";
    } else {
        std::cerr << "Some motors are not responding.\n";
    }
    std::this_thread::sleep_for(std::chrono::microseconds(5));

    act.setAngle( std::array<double, 2>{ 0, -2} );
    std::this_thread::sleep_for(std::chrono::seconds(2));

    act.updateAllFeedback();
    std::cout << "Motor 0 position: " << act.getAngle({0})[0] << '\n';


    act.setAngle( std::array<double, 2>{ M_PI_2, -2} );
    std::this_thread::sleep_for(std::chrono::seconds(2));

    act.setAngle( std::array<double, 2>{ 0, -M_PI_4 } );
    std::this_thread::sleep_for(std::chrono::seconds(2));
    act.updateAllFeedback();
    std::cout << "Motor 0 position: " << act.getAngle({0})[0] << '\n';

    act.setAngle( std::array<double, 2>{ 0, -2 } );
    std::this_thread::sleep_for(std::chrono::seconds(2));
    act.updateAllFeedback();
    std::cout << "Motor 0 position: " << act.getAngle({0})[0] << '\n';

    act.disableTorque();
    return 0;
}
