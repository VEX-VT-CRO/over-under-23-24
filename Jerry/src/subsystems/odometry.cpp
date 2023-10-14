#include <subsystems/odometry.hpp>
#include <pros/rtos.hpp>
#include <cmath>

constexpr double PI = 3.141592653589793;
constexpr double DEG2RAD = PI / 180;

Odometry::Odometry(std::uint8_t xTop, std::uint8_t xBottom, std::uint8_t yTop, 
        std::uint8_t yBottom, std::uint8_t imu, double circumference) : 
        xEncoder{pros::ADIEncoder(xTop, xBottom)}, 
        yEncoder{pros::ADIEncoder(yTop, yBottom)}, inertial{pros::Imu(imu)}
{
    position = {0, 0};
    LINEAR_MULTIPLE = PI * circumference / 360.0;
    while(inertial.is_calibrating())
    {
        pros::delay(250);
    }
}

Coordinate Odometry::getPosition()
{
    return {position.x * LINEAR_MULTIPLE, position.y * LINEAR_MULTIPLE};
}

void Odometry::setPosition(Coordinate c)
{
    position = c;
}

double Odometry::getAngle()
{
    return inertial.get_heading();
}

void Odometry::setAngle(double angle)
{
    inertial.set_heading(angle);
}

void Odometry::reset()
{
    xEncoder.reset();
    yEncoder.reset();
    inertial.reset();
}

void Odometry::update()
{
    double a = getAngle() * DEG2RAD;
    double c = std::cos(a);
    double s = std::sin(a);
    position.x += xEncoder.get_value() * s + yEncoder.get_value() * c;
    position.y += xEncoder.get_value() * c + yEncoder.get_value() * s;
    xEncoder.reset();
    yEncoder.reset();
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

    if(std::abs(target.x - pos.x) > std::abs(target.y - pos.y))
    {
        goal = target.x;
    }
    else
    {
        goal = target.y;
        useX = false;
    }

    int timer = pros::millis();
    while(pros::millis() - timer < timeout)
    {
        double actual = (useX) ? odom.getPosition().x : odom.getPosition().y;
        double error = std::abs(goal - actual);

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

        d.drive(power * 110);

        prevError = error;
        pros::delay(10);
    }

    d.drive(0);
}

void PIDController::goToAngle(TankDrivetrain &d, double target, 
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
}

void PIDController::goToTarget(TankDrivetrain &d, Coordinate target, 
        Odometry &odom, int timeout, pros::Controller& c)
{
    Coordinate pos = odom.getPosition();
    double prevError = 0;
    double nextError = 0;
    double lowerBound = 100;
    bool useX = true;
    double goal;

    if(std::abs(target.x - pos.x) > std::abs(target.y - pos.y))
    {
        goal = target.x;
    }
    else
    {
        goal = target.y;
        useX = false;
    }

    int timer = pros::millis();
    while(pros::millis() - timer < timeout && c.get_digital(pros::E_CONTROLLER_DIGITAL_B))
    {
        double actual = (useX) ? odom.getPosition().x : odom.getPosition().y;
        double error = std::abs(goal - actual);

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

        d.drive(power * 110);

        prevError = error;
        pros::delay(10);
    }

    d.drive(0);
}

void PIDController::goToAngle(TankDrivetrain &d, double target, 
        Odometry &odom, int timeout, pros::Controller& c)
{
    double prevError = 0;
    double nextError = 0;
    double lowerBound = 100;

    int timer = pros::millis();
    while(pros::millis() - timer < timeout && c.get_digital(pros::E_CONTROLLER_DIGITAL_B))
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
}