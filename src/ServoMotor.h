#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include "SMS_STS.h"
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <array>
#include <iostream>

namespace Actuator {

/**
 * Servo motor command parameters.
 * 
 * @param targetPosition: (int16_t) the target position of the servo.
 * @param targetSpeed: (int16_t) the target speed of the servo.
*/
struct ServoCommand {
    int16_t targetPosition {};
    int16_t targetSpeed {};
};


struct ServoState {
    ServoFeedback feedback{};
    ServoCommand command{};
    OperationMode mode { OperationMode::Position_servo };
};

/**
 * @brief Servo motor class. Initialize with serial number, ID, and calibration parameters. Assign driver after initialization.
 */
struct ServoMotor {
    int serialNum;
    uint8_t ID;
    int32_t zeroStep { 2047 };
    double minAngle { 0.0 };
    double maxAngle { 0.0 };
    bool posDirection { false };
    ServoFeedback m_feedback{};
    OperationMode m_mode { OperationMode::Position_servo };
    ServoCommand m_command{};

    // static variables
    static constexpr double resolution { 2 * M_PI / 4096.0 };
    static constexpr double invResolution { 2048.0 / M_PI };

    constexpr bool contains (double angle) const noexcept {
        return angle >= minAngle && angle <= maxAngle;
    }
    int32_t angleToStep(double angle) const noexcept {
        const int32_t delta { static_cast<int32_t>( std::round( angle * invResolution) ) };
        return posDirection ? zeroStep + delta : zeroStep - delta;
    }
    double stepToAngle(int32_t step) const noexcept {
        const int32_t delta = posDirection ? step - zeroStep : zeroStep - step ;
        return static_cast<double>(delta) * resolution;
    }
};

std::ostream& operator<<(std::ostream& out, const ServoMotor& motor) {
    out << "ServoMotor #" << motor.serialNum << " (ID: " << static_cast<int>(motor.ID) << "): "
        << "ZeroStep: " << motor.zeroStep << ", "
        << "MinAngle: " << motor.minAngle << ", "
        << "MaxAngle: " << motor.maxAngle << ", "
        << "PosDirection: " << (motor.posDirection ? "true" : "false");
    return out;
}


    // /**
    //  * ping the servo motor to check if it is responding.
    //  * return true if the servo motor is responding, false otherwise.
    //  */
    // bool selfInspect() {
    //     ReadResult<bool> result = m_driver.ping(m_ID);
    //     return !!result.okay;
    // }

    // /**
    //  * enable torque for the servo motor.
    //  * return true if the torque is successfully enabled, false otherwise.
    //  */
    // bool enableTorque() {
    //     ReadResult<bool> result { m_driver.enableTorque(m_ID) };
    //     return !!result.okay;
    // }

    // /**
    //  * disable torque for the servo motor.
    //  * return true if the torque is successfully disabled, false otherwise.
    //  */
    // bool disableTorque() {
    //     ReadResult<bool> result { m_driver.disableTorque(m_ID) };
    //     return !!result.okay;
    // }

    // /**
    //  * update the feedback of the servo motor, including position, speed, load, voltage, temperature, etc.
    //  * return true if the feedback is successfully updated, false otherwise.
    // */
    // bool updateFeedback() {
    //     ServoFeedback feedback { m_driver.readFeedback( m_ID ) };
    //     if ( feedback.status != ReadStatus::Okay ) {
    //         m_feedback.status = feedback.status;
    //         return false;
    //     }
    //     m_feedback = feedback;
    //     return true;
    // }

    // /**
    //  * Set the target angle (rad) of the servo motor.
    //  */
    // bool setAngle(double angle) {
    //     if ( !m_calibration.inAngleLimit(angle) ) {
    //         std::cerr << "Angle " << angle << " is out of limit for motor # " << m_serialNum << '\n';
    //         return false;
    //     }
    //     int32_t step { m_calibration.angleToStep(angle) };
    //     ReadResult<bool> result = m_driver.writePosition(m_ID, static_cast<s16>(step));
    //     return !!result.okay;
    // }

    // bool setSpeed(int speed) {
    //     ReadResult<bool> result = m_driver.writeSpeed(m_ID, static_cast<s16>(speed));
    //     return !!result.okay;
    // }

    // /**
    //  * Set the target angle (rad) of the servo motor to register.
    //  * return true if the command is successfully sent, false otherwise.
    //  */
    // bool setAngleRegister(double angle) {
    //     int32_t step { m_calibration.angleToStep(angle) };
    //     if ( !m_calibration.inAngleLimit(step) )
    //         return false;
    //     ReadResult<bool> result = m_driver.writePositionRegister(m_ID, static_cast<s16>(step));
    //     return !!result.okay;
    // }
    // double getAngle() const {
    //     return m_calibration.stepToAngle(m_feedback.position);
    // }
    // int getSpeed() const {
    //     return m_feedback.speed;
    // }
    // double getLoad() const {
    //     return m_feedback.load;
    // }
    // double getVoltage() const {
    //     return m_feedback.voltage;
    // }
    // int getTemperature() const {
    //     return m_feedback.temperature;
    // }
}      // namespace Actuator
#endif