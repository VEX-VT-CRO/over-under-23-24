#include "subsystems/turret.hpp"
#include <cmath>

Turret::Turret(pros::Motor& motor, pros::IMU& gyro, PIDConstants PID) : turretMotor{motor}, kPID{PID}, imu{gyro}
{
    prevError = 0;
    totalError = 0;
}


void Turret::turnVoltage(int mV)
{
    turretMotor.move_voltage(mV);
}

void Turret::turnAngle(int degrees)
{

}

void Turret::aimAt(Coordinate target, Coordinate pos)
{
    //TODO: Place all of this in a loop

    //Step 1: Have a target (the goal), this is the parameter
    //Step 2: Find the angle to the target relative to the robot/current turret angle
    double goalAngle = DEG2RAD * atan2(target.y - pos.y, target.x - pos.x);
    double error = goalAngle - imu.get_heading();

    double derivError = error - prevError;
    totalError += error;
    //Step 3: Move a particular direction based on angle difference (using PID)

    double power = kPID.P * error + kPID.I * totalError + kPID.D * derivError;

    turretMotor.move_voltage(power * 110);

    prevError = error;
}

void Turret::updatePosition(Coordinate goal, Coordinate pos)
{
    //Step 1: Have a target (the goal), this is the parameter
    //Step 2: Find the angle to the target relative to the robot/current turret angle
    double goalAngle = DEG2RAD * atan2(goal.y - pos.y, goal.x - pos.x);
    double error = goalAngle - imu.get_heading();

    double derivError = error - prevError;
    totalError += error;
    //Step 3: Move a particular direction based on angle difference (using PID)

    double power = kPID.P * error + kPID.I * totalError + kPID.D * derivError;

    turretMotor.move_voltage(power * 110);

    prevError = error;
}