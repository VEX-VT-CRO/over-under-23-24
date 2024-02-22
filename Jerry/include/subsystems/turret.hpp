#ifndef TURRET_HPP
#define TURRET_HPP

#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "misc.hpp"
#include "lemlib/api.hpp"

class Turret
{
    public:
        Turret(pros::Motor& motor1, pros::Motor& motor2, pros::Rotation& rotated, pros::IMU& gyro, lemlib::Chassis& chassis, lemlib::Pose target1);

        //When in autoAim mode, updatePosition does what it says
        void updatePosition();
        
        //Specifies how much the turret should be powered
        void turnVoltage(int mV);
        //Turns the turret by a specific amount of angles
        void turnAngle(int degrees);
        //Turns the turret to a particular point on the field (blocking function)
        void aimAt(Coordinate target, Coordinate pos);

        void reset_angles();

        void rotateback();

        double rotation;
        double last_rotation;
        void checkRotation();

    private:
        pros::Motor& turretMotor1, turretMotor2;
        PIDConstants kPID;
        pros::IMU& imu;
        pros::Rotation rotate;
        int prevError, totalError;
        lemlib::Chassis& chassis_bot;
        lemlib::Pose target;
        double offset_angle;
        double turn_angle;
};

#endif