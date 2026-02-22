#include "Actuator_Handler.h"
#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

int main() {

    SMS_STS st{"/dev/ttyACM0", BaudRate::r_1M};

    using namespace Actuator;
    Actuator_Handler<2> act {
        std::array<ServoMotor, 2> {
            ServoMotor{
                4,
                ServoCalibration{ .correction = 2047, .targetAngleLimit = {0, 4095}, .posDirection = false },
                st
            },
            ServoMotor{
                5,
                ServoCalibration{ .correction = 2047, .targetAngleLimit = {0, 4095}, .posDirection = false },
                st
            }
        },
        st
    };

    // self inspection
    if ( act.selfInspect() ) {
        std::cout << "All motors are responding.\n";
    } else {
        std::cerr << "Some motors are not responding.\n";
    }
    std::this_thread::sleep_for(std::chrono::microseconds(5));

    act.setAngle( 0 );
    std::this_thread::sleep_for(std::chrono::seconds(2));

    act.updateAllFeedback();
    std::cout << "Motor 0 position: " << act.getAngle({0})[0] << ", Motor 1 position: " << act.getAngle({1})[0] << '\n';


    act.setAngle( M_PI_2 );
    std::this_thread::sleep_for(std::chrono::seconds(2));

    act.updateAllFeedback();
    std::cout << "Motor 0 position: " << act.getAngle({0})[0] << ", Motor 1 position: " << act.getAngle({1})[0] << '\n';
    
    act.setAngle( -3 );
    std::this_thread::sleep_for(std::chrono::seconds(2));

    act.updateAllFeedback();
    std::cout << "Motor 0 position: " << act.getAngle({0})[0] << ", Motor 1 position: " << act.getAngle({1})[0] << '\n';

    act.setAngle( 3 );
    std::this_thread::sleep_for(std::chrono::seconds(2));

    act.updateAllFeedback();
    std::cout << "Motor 0 position: " << act.getAngle({0})[0] << ", Motor 1 position: " << act.getAngle({1})[0] << '\n';
    act.setAngle( 0 );
    return 0;
}
