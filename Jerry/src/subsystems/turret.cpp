#include "subsystems/turret.hpp"
#include <cmath>
#include "lemlib/api.hpp"

Turret::Turret(pros::Motor& motor1, pros::Motor& motor2, pros::IMU& gyro) : turretMotor1{motor1}, turretMotor2{motor2}, imu{gyro}
{

}


void Turret::turnVoltage(int mV)
{
    turretMotor1.move_voltage(mV);
    turretMotor2.move_voltage(mV);
}

void Turret::turnAngle(int degrees)
{
    uint32_t startTime = pros::millis();
    while (true) {
        if (pros::millis() - startTime > 200*abs(degrees)) {
            turretMotor1.move_voltage(0);
            turretMotor2.move_voltage(0);
            break;
        }
        turretMotor1.move_voltage(-6000*degrees/abs(degrees));
        turretMotor2.move_voltage(-6000*degrees/abs(degrees));
    }
}

void Turret::updatePosition(lemlib::Pose targetpos, lemlib::Pose currentpos)
{
    double goal_Angle = -DEG2RAD * atan2(targetpos.y - currentpos.y, targetpos.x - currentpos.x);
    double rotated_angle = imu.get_rotation();
    double turn_angle = goal_Angle + rotated_angle;
    Turret::turnAngle(turn_angle);
}