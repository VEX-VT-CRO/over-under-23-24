#include "subsystems/odometry.hpp"
#include "pros/rtos.hpp"
#include "pros/imu.h"
#include "pros/misc.hpp"
#include <cmath>

constexpr double DT = 0.01; //s
constexpr double G  = 9.81; //m/s^2 

Odometry::Odometry(std::uint8_t xGyro_port, std::uint8_t yGyro_port, std::uint8_t zGyro_port) : 
        xGyro{pros::Imu(xGyro_port)}, yGyro{pros::Imu(yGyro_port)}, zGyro{pros::Imu(zGyro_port)}
{
    position = {0, 0, 0};
    velocity = {0, 0, 0};
    acceleration = {0, 0, 0};
    XYangle = 0.0;

    while(xGyro.is_calibrating() && yGyro.is_calibrating() && zGyro.is_calibrating())
    {
        pros::delay(250);
    }
}

Coordinate Odometry::getPosition()
{
    return position;
}

void Odometry::setPosition(Coordinate c)
{
    position = c;
}

void Odometry::reset()
{
    xGyro.reset();
    yGyro.reset();
    zGyro.reset();
}

void Odometry::update()
{
    transformAcceleration();
    updateVelocity();
    updatePosition();
}

Coordinate Odometry::transformAcceleration() {
    double ax = getXAccel();
    double ay = getYAccel();
    double az = getZAccel();
    
    double yaw = getYaw();
    double pitch = getPitch();
    double roll = getRoll();

    double cosyaw = cos(yaw);
    double sinyaw = sin(yaw);
    double cospitch = cos(pitch);
    double sinpitch = sin(pitch);
    double cosroll = cos(roll);
    double sinroll = sin(roll);

    double r11 = cosyaw * cospitch;
    double r12 = cosyaw * sinpitch * sinroll - sinyaw * cosroll;
    double r13 = cosyaw * sinpitch * cosroll + sinyaw * sinroll;
    
    double r21 = sinyaw * cospitch;
    double r22 = sinyaw * sinpitch * sinroll + cosyaw * cosroll;
    double r23 = sinyaw * sinpitch * cosroll - cosyaw * sinroll;
    
    double r31 = -sinpitch;
    double r32 = cospitch * sinroll;
    double r33 = cospitch * cosroll;

    acceleration.x = r11 * ax + r12 * ay + r13 * az;
    acceleration.y = r21 * ax + r22 * ay + r23 * az;
    acceleration.z = r31 * ax + r32 * ay + r33 * az;

    double magnitude = sqrt(r11 * r11 + r21 * r21);
    XYangle = acos(r11 / magnitude);

    return acceleration;
}

Coordinate Odometry::updateVelocity() {
        velocity.x += acceleration.x * DT;
        velocity.y += acceleration.y * DT;
        velocity.z += acceleration.z * DT;
    }

Coordinate Odometry::updatePosition() {
    position.x += velocity.x * DT + 0.5 * acceleration.x * DT * DT;
    position.y += velocity.y * DT + 0.5 * acceleration.y * DT * DT;
    position.z += velocity.z * DT + 0.5 * acceleration.z * DT * DT;
    //TODO: Need to account for acceleration due to gravity
    }

double Odometry::getYaw()
{
    return xGyro.get_heading();
}

double Odometry::getPitch()
{
    return yGyro.get_heading();
}

double Odometry::getRoll()
{
    return zGyro.get_heading();
}

double Odometry::getXAccel()
{
    pros::c::imu_accel_s_t accel = xGyro.get_accel();
    return accel.x * G;
}

double Odometry::getYAccel()
{
    pros::c::imu_accel_s_t accel = yGyro.get_accel();
    return accel.x * G;
}

double Odometry::getZAccel()
{
    pros::c::imu_accel_s_t accel = zGyro.get_accel();
    return accel.x * G;
}

void Odometry::setPitch(double angle)
{
    yGyro.set_heading(angle);
}

void Odometry::setYaw(double angle)
{
    zGyro.set_heading(angle);
}

void Odometry::setRoll(double angle)
{
    xGyro.set_heading(angle);
}

PIDController::PIDController(PIDConstants pidc)
{
    this->pid = pidc;
}

void PIDController::setPID(PIDConstants pidc)
{
    this->pid = pidc;
}

void PIDController::goToTarget(TankDrivetrain &d, Coordinate target, 
        Odometry &odom, int timeout)
{
    Coordinate pos = odom.getPosition();
    double prevError = 0;
    double nextError = 0;
    double lowerBound = 100;
    bool useX = true;
    double goal;

    //Since the target is on a 2D plane, figure out which dimension to use as the target,
    //rather than computing the absolute distance.
    if(std::abs(target.x - pos.x) > std::abs(target.y - pos.y))
    {
        goal = target.x;
    }
    else
    {
        goal = target.y;
        useX = false;
    }

    //Start the clock, the difference in start time and current time must
    //be less than the timeout to make sure we don't spend too much time
    //blocking the program execution.
    int timer = pros::millis();
    while(pros::millis() - timer < timeout)
    {
        //This chooses what our measured value is, based on the conditional block above.
        double actual = (useX) ? odom.getPosition().x : odom.getPosition().y;
        double error = std::abs(goal - actual);

        //This is the error threshold for what is acceptable as "close enough".
        if(error < 0.1)
        {
            break;
        }

        //Derivative is a prediction of what will happen in the future.
        double derivError = error - prevError;

        //Integral calculation for what has happened in the past.
        nextError += error;

        //The magic happens here: the values are added together to determine
        //how much power the robot should receive to reach its goal.
        //We attenuate how much influence each factor has by multiplying PID
        //by a constant.
        double power = pid.P * error + pid.I * nextError + pid.D * derivError;

        //If the power is too low, the robot won't move and nothing happens.
        //Technically, integral can fix this on its own but that's a lot of work 
        //and introduces more issues than it solves.
        if(power < lowerBound)
        {
            power = lowerBound;
        }

        //Power is multiplied into units of mV to provide more accurate control.
        d.drive(power * 110);

        //Remember what the last error was for derivative calculations in the next iteration.
        prevError = error;
        pros::delay(10);
    }

    //Tell the robot to stop moving.
    d.drive(0);
}

/*void PIDController::goToAngle(TankDrivetrain &d, double target, 
        Odometry &odom, int timeout)
{
    double prevError = 0;
    double nextError = 0;
    double lowerBound = 100;

    int timer = pros::millis();
    while(pros::millis() - timer < timeout)
    {
        double error = target - odom.getAngle();

        if(error < 0.1)
        {
            break;
        }

        double derivError = error - prevError;
        nextError += error;

        double power = pid.P * error + pid.I * nextError + pid.D * derivError;

        if(power < lowerBound)
        {
            power = lowerBound;
        }

        d.turnLeft(power * 110);

        prevError = error;
        pros::delay(10);
    }

    d.turnLeft(0);
}*/