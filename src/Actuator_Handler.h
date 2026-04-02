#ifndef ACTUATOR_HANDLER_H
#define ACTUATOR_HANDLER_H

#include "ServoMotor.h"
#include <array>
#include <vector>
#include <string>
#include <utility>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include "joint_yaml_loader.h"
#include <iostream>

namespace Actuator {

struct PID_Gains {
    int P {};
    int I {};
    int D {};
};

template <std::size_t numMotor>
class Actuator_Handler {
public:
    // Actuator_Handler(std::array<ServoMotor, numMotor> motors, SMS_STS& st);
    Actuator_Handler(const std::array<ServoMotor, numMotor>& motors, SMS_STS& st);

public:
    /**
     * return true if all motors are responding, false otherwise.
     */
    bool selfInspect();

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
    bool setAngle(double angle);
    bool setAngle(const std::array<double, numMotor>& angle_list);
    bool setAngle(const std::vector<int>& index_list, const std::vector<double>& angle_list);

    bool setAngleRegister(const std::array<double, numMotor>& angle_list, bool pending);
    /**
     * set action position for a subset of motors. Action command is sent if pending is false.
     * @param index_list: the index of the motors to be set
     * @param position_list: the target position of the motors to be set
     * @param pending: whether to send action command after setting the position register.
     * return true if all motors are successfully set, false otherwise.
     */
    bool setAngleRegister(const std::vector<int>& index_list, const std::vector<double>& position_list, bool pending);
    bool actionTrigger();

    bool setSpeed(int speed);
    bool setSpeed(const std::vector<int>& index_list, const std::vector<int>& speed_list);

    bool updateAllFeedback();
    bool updateFeedback(int index);

    std::array<double, numMotor> getAllAngle();
    std::vector<double> getAngle(const std::vector<int>& indices);

    std::array<int, numMotor> getAllSpeed();
    std::vector<int> getSpeed(const std::vector<int>& indices);

    std::array<double, numMotor> getAllLoad();
    std::vector<double> getLoad(const std::vector<int>& indices);

    std::array<double, numMotor> getAllVoltage();
    std::vector<double> getVoltage(const std::vector<int>& indices);

    std::array<int, numMotor> getAllTemperature();
    std::vector<int> getTemperature(const std::vector<int>& indices);


    void print();

    // bool configureMinMaxAngle(int index, std::pair<int, int> limit);
    // bool configurePositionPID(int index, PID_Gains gains);
    // bool configureMode(int index, OperationMode mode);

// member variables
protected:
    SMS_STS& m_ST;
    std::array<ServoMotor, numMotor> m_motors {};
    std::array<ServoState, numMotor> m_states {};

private:
    template <typename T>
    std::array<T, numMotor> getAllInfo(T (ServoState::*getter)() const);
    template <typename T>
    std::vector<T> getInfo(const std::vector<int>& index_list, T (ServoState::*getter)() const);
};


template <std::size_t numMotor>
void Actuator_Handler<numMotor>::print() {
    std::cout << "Actuator Handler with " << numMotor << " motors:\n";
    for (const ServoMotor& motor : m_motors) {
        std::cout << motor << '\n';
    }
    return;
}

template <std::size_t numMotor>
Actuator_Handler<numMotor>::Actuator_Handler(const std::array<ServoMotor, numMotor>& motors, SMS_STS& st):
    m_motors { motors },
    m_ST { st }
{
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::selfInspect() {
    bool allOkay { true };
    for (ServoMotor& motor : m_motors ) {
        if ( !m_ST.ping(motor.ID).okay ) {
            allOkay = false;
        }
    }
    return allOkay;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::enableTorque() {
    bool return_status { true };
    for (ServoMotor& motor : m_motors) {
        bool okay { m_ST.enableTorque(motor.ID)};
        if ( !okay ) {
            return_status = false;
            std::cerr << "Fail to enable torque, motor # " << motor.serialNum << '\n';
        }
    }
    return return_status;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::enableTorque(const std::vector<int>& index_list) {
    ReadResult<bool> result {};
    bool return_status { true };
    for (const int index : index_list) {
        assert( index < numMotor && "Index should not exceed number of motors");
        ServoMotor& motor { m_motors[index] };
        bool okay { m_ST.enableTorque(motor.ID) };
        if ( !okay ) {
            return_status = false;
            std::cerr << "Fail to enable torque, motor # " << motor.serialNum << '\n';
        }
    }
    return return_status;
}


template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::disableTorque() {
    bool return_status { true };
    for (ServoMotor& motor : m_motors) {
        ReadResult<bool> result { m_ST.disableTorque(motor.ID) };
        if ( !result.okay ) {
            return_status = false;
            std::cerr << "Fail to disable torque, motor # " << motor.serialNum << '\n';
        }
    }
    return return_status;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::disableTorque(const std::vector<int>& index_list) {
    ReadResult<bool> result {};
    bool return_status { true };
    for (const int index : index_list) {
        assert( index < numMotor && "Index should not exceed number of motors");
        ServoMotor& motor { m_motors[index] };
        result = m_ST.disableTorque(motor.ID);
        if ( !result.okay ) {
            return_status = false;
            std::cerr << "Fail to disable torque, motor # " << motor.serialNum << '\n';
        }
    }
    return return_status;
}


template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::setAngle(double angle) {
    bool allOkay { true };
    for (ServoMotor& motor : m_motors) {
        if ( !motor.contains(angle) ) {
            std::cerr << "Angle " << angle << " is out of limit for motor # " << motor.serialNum << '\n';
            allOkay = false;
            continue;
        }
        int32_t step { motor.angleToStep(angle) };
        ReadResult<bool> result = m_ST.writePosition(motor.ID, static_cast<s16>(step));
        if ( !result.okay ) {
            std::cerr << "Fail to set angle, motor # " << motor.serialNum << '\n';
            allOkay = false;
        }
    }
    return true;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::setAngle(const std::array<double, numMotor>& angle_list) {
    bool return_status { true };
    for (std::size_t idx {0}; idx < numMotor; ++idx ) {
        // angle and motor
        double angle { angle_list[idx] };
        ServoMotor& motor { m_motors[idx] };
        if ( !motor.contains(angle) ) {
            std::cerr << "Angle " << angle << " is out of limit for motor # " << motor.serialNum << '\n';
            return_status = false;
            continue;
        }

        // step to go
        int32_t step { motor.angleToStep(angle) };
        ReadResult<bool> result = m_ST.writePosition(motor.ID, static_cast<s16>(step));
        if ( !result.okay ) {
            return_status = false;
            std::cerr << "Fail to set angle, motor # " << motor.serialNum << '\n';
        }
    }
    return return_status;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::setAngle(const std::vector<int>& index_list, const std::vector<double>& angle_list) {
    assert( index_list.size() == angle_list.size() && "Index list and angle list should have the same size");
    bool return_status { true };
    for ( std::size_t idx { 0 }; idx < index_list.size(); ++idx ) {

        int index { index_list[idx] };
        assert( index < numMotor && "Index should not exceed number of motors");

        double angle { angle_list[idx] };
        ServoMotor& motor { m_motors[idx] };

        int32_t step { motor.angleToStep(angle) };
        ReadResult<bool> result = m_ST.writePosition(motor.ID, static_cast<s16>(step));
        if ( !result.okay ) {
            return_status = false;
            std::cerr << "Fail to set angle, motor # " << motor.serialNum << '\n';
        }
    }
    return return_status;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::setAngleRegister(const std::array<double, numMotor>& angle_list, bool pending) {
    bool return_status { true };
    for (std::size_t idx {0}; idx < numMotor; ++idx ) {
        double angle { angle_list[idx] };
        ServoMotor& motor { m_motors[idx] };

        int32_t step { motor.angleToStep(angle) };
        ReadResult<bool> result = m_ST.writePositionRegister(motor.ID, static_cast<s16>(step));
        if ( !result.okay ) {
            return_status = false;
            std::cerr << "Fail to set angle register, motor # " << motor.serialNum << '\n';
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
bool Actuator_Handler<numMotor>::setAngleRegister(const std::vector<int>& index_list, const std::vector<double>& angle_list, bool pending) {
    assert( index_list.size() == angle_list.size() && "Index list and angle list should have the same size");
    bool return_status { true };
    for ( std::size_t idx { 0 }; idx < index_list.size(); ++idx ) {
        int index { index_list[idx] };
        assert( index < numMotor && "Index should not exceed number of motors");

        double angle { angle_list[idx] };
        ServoMotor& motor { m_motors[index] };

        int32_t step { motor.angleToStep(angle) };
        ReadResult<bool> result = m_ST.writePositionRegister(motor.ID, static_cast<s16>(step));
        if ( !result.okay ) {
            return_status = false;
            std::cerr << "Fail to set angle register, motor # " << motor.serialNum << '\n';
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
    if ( !result.okay ) {
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
bool Actuator_Handler<numMotor>::setSpeed(const std::vector<int>& index_list, const std::vector<int>& speed_list) {
    assert( index_list.size() == speed_list.size() && "Index list and speed list should have the same size");
    bool return_status { true };

    for ( std::size_t idx { 0 }; idx < index_list.size(); ++idx ) {
        int index { index_list[idx] };
        assert( index < numMotor && "Index should not exceed number of motors");
        ServoMotor& motor { m_motors[index] };
        ReadResult<bool> result = m_ST.writeSpeed(motor.ID, static_cast<s16>(speed_list[idx]));
        if ( !result.okay ) {
            return_status = false;
            std::cerr << "Fail to set speed, motor # " << motor.serialNum << '\n';
        }
    }
    return return_status;
}


template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::updateAllFeedback() {
    bool updateAll { true };
    for ( int index { 0 }; index < numMotor; ++index ) {
        ServoMotor& motor { m_motors[index] };
        ServoFeedback feedback = m_ST.readFeedback(motor.ID);
        if (feedback.status != ReadStatus::Okay) {
            updateAll = false;
            std::cerr << "Fail to update feedback, motor # " << motor.serialNum << '\n';
        }
        m_states[index].feedback = feedback;
    }
    return updateAll;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::updateFeedback(int index) {
    assert(index < numMotor && "Index should not exceed number of motors");
    ServoMotor& motor { m_motors[index] };
    // return motor.updateFeedback();
    ServoFeedback feedback = m_ST.readFeedback(motor.ID);
    if (feedback.status != ReadStatus::Okay) {
        std::cerr << "Fail to update feedback, motor # " << motor.serialNum << '\n';
        return false;
    }
    m_states[index].feedback = feedback;
    return true;
}

template <std::size_t numMotor>
template <typename T>
std::array<T, numMotor> Actuator_Handler<numMotor>::getAllInfo(T (ServoState::*getter)() const) {
    std::array<T, numMotor> info_list {};
    for ( std::size_t i {0}; i < numMotor; ++i ) {
        info_list[i] = (m_states[i].*getter)();
    }
    return info_list;
}

template <std::size_t numMotor>
template <typename T>
std::vector<T> Actuator_Handler<numMotor>::getInfo(const std::vector<int>& indices, T (ServoState::*getter)() const) {
    std::vector<T> info_list {};
    info_list.reserve(indices.size());
    for ( const int& index : indices ) {
        assert( index < numMotor && "Index should not exceed number of motors");
        info_list.emplace_back( (m_states[index].*getter)() );
    }
    return info_list;
}

template <std::size_t numMotor>
std::array<double, numMotor> Actuator_Handler<numMotor>::getAllAngle() {
    std::array<double, numMotor> info_list {};
    for ( std::size_t i {0}; i < numMotor; ++i ) {
        info_list[i] = m_motors[i].stepToAngle(m_states[i].feedback.position);
    }
    return info_list;
}

template <std::size_t numMotor>
std::vector<double> Actuator_Handler<numMotor>::getAngle(const std::vector<int>& indices) {
    std::vector<double> info_list {};
    info_list.reserve(indices.size());
    for ( const int& index : indices ) {
        assert( index < numMotor && "Index should not exceed number of motors");
        info_list.emplace_back( m_motors[index].stepToAngle(m_states[index].feedback.position) );
    }
    return info_list;
}

template <std::size_t numMotor>
std::array<int, numMotor> Actuator_Handler<numMotor>::getAllSpeed() {
    return getAllInfo<int>(&ServoState::feedback.speed);
}

template <std::size_t numMotor>
std::vector<int> Actuator_Handler<numMotor>::getSpeed(const std::vector<int>& indices) {
    return getInfo<int>(indices, &ServoState::feedback.speed);
}

template <std::size_t numMotor>
std::array<double, numMotor> Actuator_Handler<numMotor>::getAllLoad() {
    return getAllInfo<double>(&ServoState::feedback.load);
}

template <std::size_t numMotor>
std::vector<double> Actuator_Handler<numMotor>::getLoad(const std::vector<int>& indices) {
    return getInfo<double>(indices, &ServoState::feedback.load);
}

template <std::size_t numMotor>
std::array<double, numMotor> Actuator_Handler<numMotor>::getAllVoltage() {
    return getAllInfo<double>(&ServoState::feedback.voltage);
}

template <std::size_t numMotor>
std::vector<double> Actuator_Handler<numMotor>::getVoltage(const std::vector<int>& indices) {
    return getInfo<double>(indices, &ServoState::feedback.voltage);
}

template <std::size_t numMotor>
std::array<int, numMotor> Actuator_Handler<numMotor>::getAllTemperature() {
    return getAllInfo<int>(&ServoState::feedback.temperature);
}

template <std::size_t numMotor>
std::vector<int> Actuator_Handler<numMotor>::getTemperature(const std::vector<int>& indices) {
    return getInfo<int>(indices, &ServoState::feedback.temperature);
}

} // namespace Actuator

#endif