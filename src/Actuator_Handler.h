#ifndef ACTUATOR_HANDLER_H
#define ACTUATOR_HANDLER_H

#include "SMS_STS.h"
#include <array>
#include <vector>
#include <string>
#include <utility>

struct PID_Gains {
    int P {};
    int I {};
    int D {};
};


template <int numMotor>
std::array<int, numMotor> array_in_range(int b);

template <int numMotor>
std::array<int, numMotor> array_in_range(int b1, int b2);

struct AngleLimitSteps {
    int minStep { 0 };
    int maxStep { 4095 };
};

struct ServoCommand {
    int targetPosition {};
    int targetSpeed {};
    AngleLimitSteps targetAngleLimit {};
};

struct ServoMotorState {
    const int ID { };
    OperationMode mode { OperationMode::Position_servo };

    ServoCommand command{};

    ServoFeedback feedback {};
};


template <int numMotor>
class Actuator_Handler {
public:
    Actuator_Handler(std::array<ServoMotorState, numMotor> motors, std::string port);
    Actuator_Handler(std::array<ServoMotorState, numMotor> motors, std::string port, BaudRate rate);

public:

public:
    // inspect all motors for abnormal status
    bool selfInsepct();

    // display load, voltage, temperature, etc
    bool display();

    bool enableTorque();
    bool enableTorque(const std::vector<int>& index_list);

    bool disableTorque();
    bool disableTorque(const std::vector<int>& index_list);

    bool setPosition(int position);
    bool setPosition(const std::vector<int>& index_list, std::vector<int> position_list);
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



protected:
    SMS_STS m_ST {"/dev/ttyACM0", BaudRate::r_1M};
    std::array<ServoMotorState, numMotor> m_motors {};
    std::array<int, numMotor> m_index_list {};

private:
    template <typename T>
    std::array<T, numMotor> getAllInfo(T ServoFeedback::*member);

    template <typename T>
    std::vector<T> getInfo(const std::vector<int>& index_list, T ServoFeedback::*member);
};


// Implementation starts here

template <int numMotor>
std::array<int, numMotor> array_in_range(int b) {
    return array_in_range<int, numMotor>(0, b);
}

template <int numMotor>
std::array<int, numMotor> array_in_range(int b1, int b2) {
    std::array<int, numMotor> arr{};
    for (int idx {0}; idx < b2-b1; ++idx ) {
        arr[idx] = idx + b1;
    } 
    return arr;
}

template <int numMotor>
Actuator_Handler<numMotor>::Actuator_Handler(std::array<ServoMotorState, numMotor> motors, std::string port) :
    Actuator_Handler{ motors, port, BaudRate::r_1M }
{
}

template <int numMotor>
Actuator_Handler<numMotor>::Actuator_Handler(std::array<ServoMotorState, numMotor> motors, std::string port, BaudRate rate) :
    m_ST{ port, rate },
    m_motors { motors }
    // m_index_list { array_in_range<int, numMotor>(numMotor) }
{
}

template <int numMotor>
bool Actuator_Handler<numMotor>::selfInsepct() {
    bool allOkay { true };
    for (ServoMotorState& motor : m_motors ) {
        ReadResult<bool> result = m_ST.ping(
            static_cast<u8>( motor.ID )
        );
        if ( !result.okay ) {
            allOkay = false;
        }
    }
    return allOkay;
}

template <int numMotor>
bool Actuator_Handler<numMotor>::enableTorque() {
    // return enableTorque(m_index_list);
    bool return_status { true };
    for (const ServoMotorState& motor : m_motors) {
        ReadResult<bool> result = m_ST.enableTorque( motor.ID );
        if ( !result.okay ) {
            return_status = false;
            std::cerr << "Fail to enable torque, motor # " << motor.ID << '\n';
        }
    }
    return return_status;
}

template <int numMotor>
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


template <int numMotor>
bool Actuator_Handler<numMotor>::disableTorque() {
    bool return_status { true };
    for (const ServoMotorState& motor : m_motors) {
        ReadResult<bool> result = m_ST.disableTorque( motor.ID );
        if ( result.status != ReadStatus::Okay ) {
            return_status = false;
        }
    }
    return return_status;
}

template <int numMotor>
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


template <int numMotor>
bool Actuator_Handler<numMotor>::setPosition(int position) {
    // Broadcast write
    ReadResult<bool> result = m_ST.writePosition(
        m_ST.s_broadcastID,
        static_cast<s16>(position)
    );
    return true;
}

template <int numMotor>
bool Actuator_Handler<numMotor>::setPosition(const std::vector<int>& index_list, std::vector<int> position_list) {
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

template <int numMotor>
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
        ReadResult<bool> result = m_ST.regWriteAction();
        if ( !result.okay ) {
            std::cerr << "Fail to trigger action.\n";
            return false;
        }
    }
    return return_status;
}

template <int numMotor>
bool Actuator_Handler<numMotor>::actionTrigger() {
    ReadResult<bool> result = m_ST.regWriteAction();
    if ( !result.status ) {
        std::cerr << "Fail to trigger action.\n";
        return false;
    }
    return true;
}

template <int numMotor>
bool Actuator_Handler<numMotor>::setSpeed(int speed) {
    // Broadcast write
    ReadResult<bool> result = m_ST.writeSpeed(
        m_ST.s_broadcastID,
        static_cast<s16>(speed)
    );
    return true;
}

template <int numMotor>
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


template <int numMotor>
bool Actuator_Handler<numMotor>::updateAllFeedback() {
    bool updateAll { true };
    for ( ServoMotorState& motor : m_motors ) {
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

template <int numMotor>
bool Actuator_Handler<numMotor>::updateFeedback(int index) {
    ServoMotorState& motor { m_motors[index] };
    ServoFeedback feedback { m_ST.readFeedback( motor.ID ) };
    if ( feedback.status != ReadStatus::Okay ) {
        motor.feedback.status = feedback.status;
        return false;
    }
    motor.feedback = feedback;
    return true;
}

template <int numMotor>
template <typename T>
std::array<T, numMotor> Actuator_Handler<numMotor>::getAllInfo(T ServoFeedback::*member) {
    std::array<T, numMotor> info_list {};
    for ( std::size_t i {0}; i < numMotor; ++i ) {
        info_list[i] = m_motors[i].feedback.*member;
    }
    return info_list;
}

template <int numMotor>
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

// TODO: what type to return here
template <int numMotor>
std::array<int, numMotor> Actuator_Handler<numMotor>::getAllPosition() {
    return getAllInfo<int>(&ServoFeedback::position);
}

template <int numMotor>
std::vector<int> Actuator_Handler<numMotor>::getPosition(const std::vector<int>& indices) {
    return getInfo<int>(indices, &ServoFeedback::position);
}

template <int numMotor>
std::array<int, numMotor> Actuator_Handler<numMotor>::getAllSpeed() {
    return getAllInfo<int>(&ServoFeedback::speed);
}

template <int numMotor>
std::vector<int> Actuator_Handler<numMotor>::getSpeed(const std::vector<int>& indices) {
    return getInfo<int>(indices, &ServoFeedback::speed);
}

template <int numMotor>
std::array<double, numMotor> Actuator_Handler<numMotor>::getAllLoad() {
    return getAllInfo<double>(&ServoFeedback::load);
}

template <int numMotor>
std::vector<double> Actuator_Handler<numMotor>::getLoad(const std::vector<int>& indices) {
    return getInfo<double>(indices, &ServoFeedback::load);
}

template <int numMotor>
std::array<double, numMotor> Actuator_Handler<numMotor>::getAllVoltage() {
    return getAllInfo<double>(&ServoFeedback::voltage);
}

template <int numMotor>
std::vector<double> Actuator_Handler<numMotor>::getVoltage(const std::vector<int>& indices) {
    return getInfo<double>(indices, &ServoFeedback::voltage);
}

template <int numMotor>
std::array<int, numMotor> Actuator_Handler<numMotor>::getAllTemperature() {
    return getAllInfo<int>(&ServoFeedback::temperature);
}

template <int numMotor>
std::vector<int> Actuator_Handler<numMotor>::getTemperature(const std::vector<int>& indices) {
    return getInfo<int>(indices, &ServoFeedback::temperature);
}

#endif