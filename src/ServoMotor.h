#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include "SMS_STS.h"
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <array>

namespace Actuator {

/**
 * Angle limit in step unit, typically 0-4095, corresponding to 0-360 degree.
 * 
 * minStep: (int32_t) the minimum step of the servo
 * maxStep: (int32_t) the maximum step of the servo
 */
struct AngleLimitSteps {
    uint16_t minStep { 0 };
    uint16_t maxStep { 4095 };
    constexpr AngleLimitSteps(uint16_t a = 0, uint16_t b = 4095) :
        minStep{ a < b ? a : b},
        maxStep{ a < b ? b : a}
    {}
    constexpr bool contains (uint16_t step) const noexcept {
        return step >= minStep && step <= maxStep;
    }
};

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


/**
 * Servo motor calibration parameters, including correction, angle limit, and position direction.
 * 
 * @param correction: (int32_t) 0-4095. Typically around 2047.
 * @param targetAngleLimit: (AngleLimitSteps) 0-4095.
 * @param posDirection: (bool) position direction.
*/
struct ServoCalibration {
    int32_t correction { 2047 };
    AngleLimitSteps targetAngleLimit {0, 4095};
    bool posDirection { false };

    static constexpr double resolution { 2 * M_PI / 4096.0 };
    static constexpr double invResolution { 2048.0 / M_PI };

    constexpr bool inAngleLimit(int32_t position) const noexcept {
        return targetAngleLimit.contains( position  );
    }
    int32_t angleToStep(double angle) const noexcept {
        const int32_t delta { static_cast<int32_t>( std::round( angle * invResolution) ) };
        return posDirection ? correction + delta : correction - delta;
    }
    double stepToAngle(int32_t step) const noexcept {
        const int32_t delta = posDirection ? step - correction : correction - step ;
        return static_cast<double>(delta) * resolution;
    }
};

class ServoMotor {
public:
    ServoMotor(const uint8_t ID, const ServoCalibration& calibration, SMS_STS& st) :
        m_ID{ ID },
        m_calibration{ calibration },
        m_driver{ st }
    {}

    /**
     * ping the servo motor to check if it is responding.
     * return true if the servo motor is responding, false otherwise.
     */
    bool selfInspect() {
        ReadResult<bool> result = m_driver.ping(m_ID);
        return !!result.okay;
    }

    /**
     * enable torque for the servo motor.
     * return true if the torque is successfully enabled, false otherwise.
     */
    bool enableTorque() {
        ReadResult<bool> result { m_driver.enableTorque(m_ID) };
        return !!result.okay;
    }

    /**
     * disable torque for the servo motor.
     * return true if the torque is successfully disabled, false otherwise.
     */
    bool disableTorque() {
        ReadResult<bool> result { m_driver.disableTorque(m_ID) };
        return !!result.okay;
    }

    /**
     * update the feedback of the servo motor, including position, speed, load, voltage, temperature, etc.
     * return true if the feedback is successfully updated, false otherwise.
    */
    bool updateFeedback() {
        ServoFeedback feedback { m_driver.readFeedback( m_ID ) };
        if ( feedback.status != ReadStatus::Okay ) {
            m_feedback.status = feedback.status;
            return false;
        }
        m_feedback = feedback;
        return true;
    }

    /**
     * Set the target angle (rad) of the servo motor.
     */
    bool setAngle(double angle) {
        int32_t step { m_calibration.angleToStep(angle) };
        if ( !m_calibration.inAngleLimit(step) )
            return false;
        ReadResult<bool> result = m_driver.writePosition(m_ID, static_cast<s16>(step));
        return !!result.okay;
    }

    bool setSpeed(int speed) {
        ReadResult<bool> result = m_driver.writeSpeed(m_ID, static_cast<s16>(speed));
        return !!result.okay;
    }

    /**
     * Set the target angle (rad) of the servo motor to register.
     * return true if the command is successfully sent, false otherwise.
     */
    bool setAngleRegister(double angle) {
        int32_t step { m_calibration.angleToStep(angle) };
        if ( !m_calibration.inAngleLimit(step) )
            return false;
        ReadResult<bool> result = m_driver.writePositionRegister(m_ID, static_cast<s16>(step));
        return !!result.okay;
    }
    double getAngle() const {
        return m_calibration.stepToAngle(m_feedback.position);
    }
    int getSpeed() const {
        return m_feedback.speed;
    }
    double getLoad() const {
        return m_feedback.load;
    }
    double getVoltage() const {
        return m_feedback.voltage;
    }
    int getTemperature() const {
        return m_feedback.temperature;
    }

public:
    int ID() const { return static_cast<int>(m_ID); }
private:
    uint8_t m_ID;
    ServoCalibration m_calibration;
    SMS_STS& m_driver;
    ServoFeedback m_feedback{};
    OperationMode m_mode { OperationMode::Position_servo };
    ServoCommand m_command{};
};


}      // namespace Actuator
#endif