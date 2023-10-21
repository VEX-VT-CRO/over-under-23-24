#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/imu.hpp"
#include "misc.hpp"
#include "drivetrain.hpp"
#include "pros/misc.hpp"

class Odometry
{
    public:
        Odometry(std::uint8_t xGyro_port, std::uint8_t yGyro_port, std::uint8_t zGyro_port);
        Coordinate getPosition();
        Coordinate updatePosition();
        Coordinate updateVelocity();
        void setPosition(Coordinate c);
        double getYaw();
        double getPitch();
        double getRoll();

        double getXAccel();
        double getYAccel();
        double getZAccel();

        Coordinate transformAcceleration(double roll, double pitch, double yaw);

        void setYaw(double angle);
        void setPitch(double angle);
        void setRoll(double angle);

        void resetTo(Coordinate c);
        void reset();
        void update();

    private:
        pros::Imu xGyro, yGyro, zGyro;
        Coordinate position;
        Coordinate velocity;
        Coordinate angle;
        Coordinate acceleration;
};

class PIDController
{
    public:
        PIDController(PIDConstants pidc);
        void setPID(PIDConstants pidc);
        void goToTarget(TankDrivetrain& d, Coordinate target, 
                Odometry& odom, int timeout);
        void goToAngle(TankDrivetrain& t, double angle, 
                Odometry& odom, int timeout);

    private:
        PIDConstants pid;
};

#endif