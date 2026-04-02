#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>
#include <stdint.h>
#include "Actuator_Handler.h"
#include <array>

// YAML: Joint format
// - Joint:
//     Number: 1
//     ID: 11
//     ZeroStep: 2014
//     MinAngle: -90
//     MaxAngle: 90
//     positive_direction: false

template <int N>
std::array<Actuator::ServoMotor, N> loadMotorsFromYAML(const std::string& filename) {
    std::array<Actuator::ServoMotor, N> motors {};
    try {
        YAML::Node root = YAML::LoadFile(filename);
        std::cout << "Loaded YAML file successfully\n";
        if ( !root["Robot"]) {
            std::cout << "Unsupported yaml format: missing Robot node\n";
            return motors;
        }
        // for (const auto& item : root["Robot"]) {
        for (int i = 0; i < N; ++i) {
            const auto& item = root["Robot"][i];
            YAML::Node joint = item["Joint"];
            int serialNum { joint["Number"].as<int>() };
            uint8_t ID { static_cast<uint8_t>(joint["ID"].as<int>()) };
            // in ServoCalibration class
            int32_t zeroStep { static_cast<int32_t>(joint["ZeroStep"].as<int>()) };
            double minAngle { joint["MinAngle"].as<double>() };
            double maxAngle { joint["MaxAngle"].as<double>() };
            bool posDirection { joint["positive_direction"].as<bool>() };
            motors[i] = Actuator::ServoMotor {
                .serialNum = serialNum,
                .ID = ID,
                .zeroStep = zeroStep,
                .minAngle = minAngle,
                .maxAngle = maxAngle,
                .posDirection = posDirection
            };
        }
    } catch(YAML::ParserException& e) {
        std::cout << e.what() << "\n";
    }
   return motors;
}


