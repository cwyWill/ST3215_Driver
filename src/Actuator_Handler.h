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

template <std::size_t numMotor>
class Actuator_Handler {
public:
    Actuator_Handler(std::array<ServoMotor, numMotor> motors, SMS_STS& st);

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



    // bool configureMinMaxAngle(int index, std::pair<int, int> limit);
    // bool configurePositionPID(int index, PID_Gains gains);
    // bool configureMode(int index, OperationMode mode);

// member variables
protected:
    SMS_STS& m_ST;
    std::array<ServoMotor, numMotor> m_motors {};

private:
    template <typename T>
    std::array<T, numMotor> getAllInfo(T (ServoMotor::*getter)() const);

    template <typename T>
    std::vector<T> getInfo(const std::vector<int>& index_list, T (ServoMotor::*getter)() const);
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
Actuator_Handler<numMotor>::Actuator_Handler(std::array<ServoMotor, numMotor> motors, SMS_STS& st) :
    m_motors { motors },
    m_ST { st }
{
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::selfInspect() {
    bool allOkay { true };
    for (ServoMotor& motor : m_motors ) {
        bool okay { motor.selfInspect() };
        if ( !okay ) {
            allOkay = false;
        }
    }
    return allOkay;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::enableTorque() {
    bool return_status { true };
    for (ServoMotor& motor : m_motors) {
        bool okay { motor.enableTorque() };
        if ( !okay ) {
            return_status = false;
            std::cerr << "Fail to enable torque, motor # " << motor.ID() << '\n';
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
        bool okay { motor.enableTorque() };
        if ( !okay ) {
            return_status = false;
            std::cerr << "Fail to enable torque, motor # " << motor.ID() << '\n';
        }
    }
    return return_status;
}


template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::disableTorque() {
    bool return_status { true };
    for (ServoMotor& motor : m_motors) {
        bool okay { motor.disableTorque() };
        if ( !okay ) {
            return_status = false;
            std::cerr << "Fail to disable torque, motor # " << motor.ID() << '\n';
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
        result = m_ST.disableTorque(m_motors[index].ID);
        if ( !result.okay ) {
            return_status = false;
            std::cerr << "Fail to disable torque, motor # " << index << '\n';
        }
    }
    return return_status;
}


template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::setAngle(double angle) {
    bool allOkay { true };
    for (ServoMotor& motor : m_motors) {
        bool okay { motor.setAngle(angle) };
        if ( !okay ) {
            std::cerr << "Fail to set angle, motor # " << motor.ID() << '\n';
            allOkay = false;
        }
    }
    return true;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::setAngle(const std::array<double, numMotor>& angle_list) {
    bool return_status { true };
    for (std::size_t idx {0}; idx < numMotor; ++idx ) {
        double angle { angle_list[idx] };
        ServoMotor& motor { m_motors[idx] };
        bool okay { motor.setAngle(angle) };
        if ( !okay ) {
            return_status = false;
            std::cerr << "Fail to set angle, motor # " << motor.ID() << '\n';
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
        ServoMotor& motor { m_motors[index] };
        bool okay { motor.setAngle(angle) };
        if ( !okay ) {
            return_status = false;
            std::cerr << "Fail to set angle, motor # " << motor.ID() << '\n';
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
        bool okay { motor.setAngleRegister(angle) };
        if ( !okay ) {
            return_status = false;
            std::cerr << "Fail to set angle register, motor # " << motor.ID() << '\n';
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
        bool okay { motor.setAngle(angle) };
        if ( !okay ) {
            return_status = false;
            std::cerr << "Fail to set angle, motor # " << motor.ID() << '\n';
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
bool Actuator_Handler<numMotor>::setSpeed(const std::vector<int>& index_list, const std::vector<int>& speed_list) {
    assert( index_list.size() == speed_list.size() && "Index list and speed list should have the same size");
    bool return_status { true };

    for ( std::size_t idx { 0 }; idx < index_list.size(); ++idx ) {
        int index { index_list[idx] };
        assert( index < numMotor && "Index should not exceed number of motors");
        ServoMotor& motor { m_motors[index] };
        bool okay { motor.setSpeed(speed_list[idx]) };
        if ( !okay ) {
            return_status = false;
            std::cerr << "Fail to set speed, motor # " << motor.ID() << '\n';
        }
    }
    return return_status;
}


template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::updateAllFeedback() {
    bool updateAll { true };
    for ( ServoMotor& motor : m_motors ) {
        bool okay { motor.updateFeedback() };
        if ( !okay ) {
            updateAll = false;
            std::cerr << "Fail to update feedback, motor # " << motor.ID() << '\n';
        }
    }
    return updateAll;
}

template <std::size_t numMotor>
bool Actuator_Handler<numMotor>::updateFeedback(int index) {
    ServoMotor& motor { m_motors[index] };
    return motor.updateFeedback();
}

template <std::size_t numMotor>
template <typename T>
std::array<T, numMotor> Actuator_Handler<numMotor>::getAllInfo(T (ServoMotor::*getter)() const) {
    std::array<T, numMotor> info_list {};
    for ( std::size_t i {0}; i < numMotor; ++i ) {
        info_list[i] = (m_motors[i].*getter)();
    }
    return info_list;
}

template <std::size_t numMotor>
template <typename T>
std::vector<T> Actuator_Handler<numMotor>::getInfo(const std::vector<int>& indices, T (ServoMotor::*getter)() const) {
    std::vector<T> info_list {};
    info_list.reserve(indices.size());
    for ( const int& index : indices ) {
        assert( index < numMotor && "Index should not exceed number of motors");
        info_list.emplace_back( (m_motors[index].*getter)() );
    }
    return info_list;
}

template <std::size_t numMotor>
std::array<double, numMotor> Actuator_Handler<numMotor>::getAllAngle() {
    return getAllInfo<double>(&ServoMotor::getAngle);
}

template <std::size_t numMotor>
std::vector<double> Actuator_Handler<numMotor>::getAngle(const std::vector<int>& indices) {
    return getInfo<double>(indices, &ServoMotor::getAngle);
}

template <std::size_t numMotor>
std::array<int, numMotor> Actuator_Handler<numMotor>::getAllSpeed() {
    return getAllInfo<int>(&ServoMotor::getSpeed);
}

template <std::size_t numMotor>
std::vector<int> Actuator_Handler<numMotor>::getSpeed(const std::vector<int>& indices) {
    return getInfo<int>(indices, &ServoMotor::getSpeed);
}

template <std::size_t numMotor>
std::array<double, numMotor> Actuator_Handler<numMotor>::getAllLoad() {
    return getAllInfo<double>(&ServoMotor::getLoad);
}

template <std::size_t numMotor>
std::vector<double> Actuator_Handler<numMotor>::getLoad(const std::vector<int>& indices) {
    return getInfo<double>(indices, &ServoMotor::getLoad);
}

template <std::size_t numMotor>
std::array<double, numMotor> Actuator_Handler<numMotor>::getAllVoltage() {
    return getAllInfo<double>(&ServoMotor::getVoltage);
}

template <std::size_t numMotor>
std::vector<double> Actuator_Handler<numMotor>::getVoltage(const std::vector<int>& indices) {
    return getInfo<double>(indices, &ServoMotor::getVoltage);
}

template <std::size_t numMotor>
std::array<int, numMotor> Actuator_Handler<numMotor>::getAllTemperature() {
    return getAllInfo<int>(&ServoMotor::getTemperature);
}

template <std::size_t numMotor>
std::vector<int> Actuator_Handler<numMotor>::getTemperature(const std::vector<int>& indices) {
    return getInfo<int>(indices, &ServoMotor::getTemperature);
}

} // namespace Actuator

#endif