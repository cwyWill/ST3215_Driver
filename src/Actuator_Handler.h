#ifndef ACTUATOR_HANDLER_H
#define ACTUATOR_HANDLER_H

#include "SMS_STS.h"
#include <array>
#include <vector>
#include <string>
#include <utility>
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace Actuator {

struct PID_Gains {
    int P {};
    int I {};
    int D {};
};


template <std::size_t numMotor>
std::array<int, numMotor> array_in_range(int b);

template <std::size_t numMotor>
std::array<int, numMotor> array_in_range(int b1, int b2);

/**
 * Angle limit in step unit, typically 0-4095, corresponding to 0-360 degree.
 * 
 * minStep: (int32_t) the minimum step of the servo
 * maxStep: (int32_t) the maximum step of the servo
 */
struct AngleLimitSteps {
    int32_t minStep { 0 };
    int32_t maxStep { 4095 };
    constexpr AngleLimitSteps(int32_t a = 0, int32_t b = 4095) :
        minStep{ a < b ? a : b},
        maxStep{ a < b ? b : a}
    {}
    constexpr bool contains (int32_t step) const noexcept {
        return step >= minStep && step <= maxStep;
    }
};

/**
 * Servo motor command parameters.
 * 
 * targetPosition: (int16_t) the target position of the servo.
 * targetSpeed: (int16_t) the target speed of the servo.
*/
struct ServoCommand {
    int16_t targetPosition {};
    int16_t targetSpeed {};
};


/**
 * Servo motor calibration parameters, including correction, angle limit, and position direction.
 * 
 * correction: (int32_t) 0-4095. Typically around 2047.
 * targetAngleLimit: (AngleLimitSteps) 0-4095.
 * posDirection: (bool) position direction.
*/
struct ServoCalibration {

    int32_t correction { 2047 };
    AngleLimitSteps targetAngleLimit {0, 4095};
    bool posDirection { false };

    static constexpr double resolution { M_PI / 2048.0 };
    static constexpr double invResolution { 2048.0 / M_PI };

    constexpr bool inAngleLimit(int32_t position) const noexcept {
        return targetAngleLimit.contains( position  );
    }
    int32_t angleToStep(double angle) const noexcept {
        const int32_t delta { static_cast<int32_t>( std::round( angle * invResolution) ) };
        return posDirection ? correction - delta : correction + delta;
    }
    double stepToAngle(int32_t step) const noexcept {
        const int32_t delta = posDirection ? correction - step : step - correction;
        return static_cast<double>(delta) * resolution;
    }
};

/**
 * Servo motor state, including ID, calibration parameters, feedback, command, and working mode.
 * ID: (uint8_t) the ID of the servo.
 * calibration: (ServoCalibration) the calibration parameters of the servo.
 * feedback: (ServoFeedback) the feedback from the servo, including position, speed, load, ...
 * command: (ServoCommand) the command to be sent to the servo, including target position and speed.
 * mode: (OperationMode) the working mode of the servo, including position servo, speed servo, etc.
 */
struct ServoMotor {
    const uint8_t ID { };
    ServoCalibration calibration {};
    ServoFeedback feedback {};
    ServoCommand command{};
    OperationMode mode { OperationMode::Position_servo };
};


template <std::size_t numMotor>
class Actuator_Handler {
public:
    Actuator_Handler(std::array<ServoMotor, numMotor> motors, std::string port);
    Actuator_Handler(std::array<ServoMotor, numMotor> motors, std::string port, BaudRate rate);

public:
    /**
     * return true if all motors are responding, false otherwise.
     */
    bool selfInsepct();

    /**
     * display the feedback of all motors, including position, speed, load, voltage, temperature, etc.
     */
    bool display();

    /**
     * enable torque for all motors or a subset of motors.
     * return true if all motors are successfully enabled, false otherwise.
     */
    bool enableTorque();
    bool enableTorque(const std::vector<int>& index_list);

    /**
     * disable torque for all motors or a subset of motors.
     * return true if all motors are successfully disabled, false otherwise.
     */
    bool disableTorque();
    bool disableTorque(const std::vector<int>& index_list);

    /**
     * set position for all motors or a subset of motors.
     * return true if all motors are successfully set, false otherwise.
     */
    bool setPosition(int position);
    bool setPosition(const std::array<int, numMotor>& position_list);
    bool setPosition(const std::vector<int>& index_list, const std::vector<int> position_list);

    /**
     * set action position for a subset of motors. Action command is sent if pending is false.
     * @param index_list: the index of the motors to be set
     * @param position_list: the target position of the motors to be set
     * @param pending: whether to send action command after setting the position register.
     * return true if all motors are successfully set, false otherwise.
     */
    bool setPositionRegister(const std::vector<int>& index_list, std::vector<int> position_list, bool pending = false);
    bool actionTrigger();

    bool setSpeed(int speed);
    bool setSpeed(const std::vector<int>& index_list, std::vector<int> speed_list);

    bool updateAllFeedback();
    bool updateFeedback(int index);

    std::array<int, numMotor> getAllPosition();
    std::vector<int> getPosition(const std::vector<int>& indices);

    std::array<int, numMotor> getAllSpeed();
    std::vector<int> getSpeed(const std::vector<int>& indices);

    std::array<double, numMotor> getAllLoad();
    std::vector<double> getLoad(const std::vector<int>& indices);

    std::array<double, numMotor> getAllVoltage();
    std::vector<double> getVoltage(const std::vector<int>& indices);

    std::array<int, numMotor> getAllTemperature();
    std::vector<int> getTemperature(const std::vector<int>& indices);



    bool configureMinMaxAngle(int index, std::pair<int, int> limit);
    bool configurePositionPID(int index, PID_Gains gains);
    bool configureMode(int index, OperationMode mode);

// member variables
protected:
    SMS_STS m_ST {"/dev/ttyACM0", BaudRate::r_1M};
    std::array<ServoMotor, numMotor> m_motors {};

private:
    template <typename T>
    std::array<T, numMotor> getAllInfo(T ServoFeedback::*member);

    template <typename T>
    std::vector<T> getInfo(const std::vector<int>& index_list, T ServoFeedback::*member);
};


// Implementation starts here

template <std::size_t numMotor>
std::array<int, numMotor> array_in_range(int b) {
    return array_in_range<int, numMotor>(0, b);
}

template <std::size_t numMotor>
std::array<int, numMotor> array_in_range(int b1, int b2) {
    std::array<int, numMotor> arr{};
    for (int idx {0}; idx < b2-b1; ++idx ) {
        arr[idx] = idx + b1;
    } 
    return arr;
}

template <std::size_t numMotor>
Actuator_Handler<numMotor>::Actuator_Handler(std::array<ServoMotor, numMotor> motors, std::string port) :
    Actuator_Handler{ motors, port, BaudRate::r_1M }
{
}

template <std::size_t numMotor>
Actuator_Handler<numMotor>::Actuator_Handler(std::array<ServoMotor, numMotor> motors, std::string port, BaudRate rate) :
    m_ST{ port, rate },
    m_motors { motors }
{
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::selfInsepct() {
    bool allOkay { true };
    for (ServoMotor& motor : m_motors ) {
        ReadResult<bool> result = m_ST.ping(
            motor.ID
        );
        if ( !result.okay ) {
            allOkay = false;
        }
    }
    return allOkay;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::enableTorque() {
    bool return_status { true };
    for (const ServoMotor& motor : m_motors) {
        ReadResult<bool> result = m_ST.enableTorque( motor.ID );
        if ( !result.okay ) {
            return_status = false;
            std::cerr << "Fail to enable torque, motor # " << motor.ID << '\n';
        }
    }
    return return_status;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::enableTorque(const std::vector<int>& index_list) {
    ReadResult<bool> result {};
    bool return_status { true };
    for (const int index : index_list) {
        result = m_ST.enableTorque( m_motors[index].ID );
        if ( result.status != ReadStatus::Okay ) {
            return_status = false;
            std::cerr << "Fail to enable torque, motor # " << index << '\n';
        }
    }
    return return_status;
}


template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::disableTorque() {
    bool return_status { true };
    for (const ServoMotor& motor : m_motors) {
        ReadResult<bool> result = m_ST.disableTorque( motor.ID );
        if ( result.status != ReadStatus::Okay ) {
            return_status = false;
        }
    }
    return return_status;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::disableTorque(const std::vector<int>& index_list) {
    ReadResult<bool> result {};
    bool return_status { true };
    for (const int index : index_list) {
        result = m_ST.disableTorque(m_motors[index].ID);
        if ( !result.okay ) {
            return_status = false;
            std::cerr << "Fail to disable torque, motor # " << index << '\n';
        }
    }
    return return_status;
}


template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::setPosition(int position) {
    // Broadcast write
    ReadResult<bool> result = m_ST.writePosition(
        m_ST.s_broadcastID,
        static_cast<s16>(position)
    );
    return true;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::setPosition(const std::vector<int>& index_list, const std::vector<int> position_list) {
    assert( index_list.size() == position_list.size() && "Index list and position list should have the same size");
    ReadResult<bool> result {};
    bool return_status { true };

    for ( std::size_t idx { 0 }; idx < index_list.size(); ++idx ) {
        int index { index_list[idx] };
        assert( index < numMotor && "Index should not exceed number of motors");

        int ID { m_motors[index].ID };
        int position { position_list[idx] };

        result = m_ST.writePosition(
            static_cast<u8>(ID),
            static_cast<s16>(position)
        );

        if ( result.status != ReadStatus::Okay ) {
            return_status = false;
            std::cerr << "Fail to assign position, motor # " << index << '\n';
        }
    }
    return return_status;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::setPositionRegister(const std::vector<int>& index_list, std::vector<int> position_list, bool pending) {
    assert( index_list.size() == position_list.size() && "Index list and position list should have the same size");
    bool return_status { true };
    for ( std::size_t idx { 0 }; idx < index_list.size(); ++idx ) {
        int index { index_list[idx] };
        assert( index < numMotor && "Index should not exceed number of motors");

        int ID { m_motors[index].ID };
        int position { position_list[idx] };

        ReadResult<bool> result = m_ST.writePositionRegister(
            ID,
            static_cast<s16>(position)
        );

        if ( !result.status ) {
            std::cerr << "Fail to assign position register, motor # " << index << '\n';
            return_status = false;
        }
    }
    if ( !pending ) {
        if ( !actionTrigger() ) {
            return false;
        }
    }
    return return_status;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::actionTrigger() {
    ReadResult<bool> result = m_ST.regWriteAction();
    if ( !result.status ) {
        std::cerr << "Fail to trigger action.\n";
        return false;
    }
    return true;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::setSpeed(int speed) {
    // Broadcast write
    ReadResult<bool> result = m_ST.writeSpeed(
        m_ST.s_broadcastID,
        static_cast<s16>(speed)
    );
    return true;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::setSpeed(const std::vector<int>& index_list, std::vector<int> speed_list) {
    assert( index_list.size() == speed_list.size() && "Index list and speed list should have the same size");
    ReadResult<bool> result {};
    bool return_status { true };

    for ( std::size_t idx { 0 }; idx < index_list.size(); ++idx ) {
        int index { index_list[idx] };
        assert( index < numMotor && "Index should not exceed number of motors");

        int speed { speed_list[idx] };

        result = m_ST.writeSpeed(
            m_motors[index].ID,
            static_cast<s16>(speed)
        );

        if ( result.status != ReadStatus::Okay ) {
            return_status = false;
            std::cerr << "Fail to assign speed, motor # " << index << '\n';
        }
    }
    return return_status;
}


template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::updateAllFeedback() {
    bool updateAll { true };
    for ( ServoMotor& motor : m_motors ) {
        ServoFeedback feedback { m_ST.readFeedback( motor.ID ) };
        if ( feedback.status != ReadStatus::Okay ) {
            motor.feedback.status == feedback.status;
            updateAll = false;
            continue;
        }
        motor.feedback = feedback;
    }
    return updateAll;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::updateFeedback(int index) {
    ServoMotor& motor { m_motors[index] };
    ServoFeedback feedback { m_ST.readFeedback( motor.ID ) };
    if ( feedback.status != ReadStatus::Okay ) {
        motor.feedback.status = feedback.status;
        return false;
    }
    motor.feedback = feedback;
    return true;
}

template <std::size_t numMotor>
template <typename T>
std::array<T, numMotor> Actuator_Handler<numMotor>::getAllInfo(T ServoFeedback::*member) {
    std::array<T, numMotor> info_list {};
    for ( std::size_t i {0}; i < numMotor; ++i ) {
        info_list[i] = m_motors[i].feedback.*member;
    }
    return info_list;
}

template <std::size_t numMotor>
template <typename T>
std::vector<T> Actuator_Handler<numMotor>::getInfo(const std::vector<int>& indices, T ServoFeedback::*member) {
    std::vector<T> info_list {};
    info_list.reserve(indices.size());
    for ( const int& index : indices ) {
        assert( index < numMotor && "Index should not exceed number of motors");
        info_list.emplace_back(m_motors[index].feedback.*member);
    }
    return info_list;
}

template <std::size_t numMotor>
std::array<int, numMotor> Actuator_Handler<numMotor>::getAllPosition() {
    return getAllInfo<int>(&ServoFeedback::position);
}

template <std::size_t numMotor>
std::vector<int> Actuator_Handler<numMotor>::getPosition(const std::vector<int>& indices) {
    return getInfo<int>(indices, &ServoFeedback::position);
}

template <std::size_t numMotor>
std::array<int, numMotor> Actuator_Handler<numMotor>::getAllSpeed() {
    return getAllInfo<int>(&ServoFeedback::speed);
}

template <std::size_t numMotor>
std::vector<int> Actuator_Handler<numMotor>::getSpeed(const std::vector<int>& indices) {
    return getInfo<int>(indices, &ServoFeedback::speed);
}

template <std::size_t numMotor>
std::array<double, numMotor> Actuator_Handler<numMotor>::getAllLoad() {
    return getAllInfo<double>(&ServoFeedback::load);
}

template <std::size_t numMotor>
std::vector<double> Actuator_Handler<numMotor>::getLoad(const std::vector<int>& indices) {
    return getInfo<double>(indices, &ServoFeedback::load);
}

template <std::size_t numMotor>
std::array<double, numMotor> Actuator_Handler<numMotor>::getAllVoltage() {
    return getAllInfo<double>(&ServoFeedback::voltage);
}

template <std::size_t numMotor>
std::vector<double> Actuator_Handler<numMotor>::getVoltage(const std::vector<int>& indices) {
    return getInfo<double>(indices, &ServoFeedback::voltage);
}

template <std::size_t numMotor>
std::array<int, numMotor> Actuator_Handler<numMotor>::getAllTemperature() {
    return getAllInfo<int>(&ServoFeedback::temperature);
}

template <std::size_t numMotor>
std::vector<int> Actuator_Handler<numMotor>::getTemperature(const std::vector<int>& indices) {
    return getInfo<int>(indices, &ServoFeedback::temperature);
}

} // namespace Actuator

#endif