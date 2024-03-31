#include "main.h"

#include "lemlib/api.hpp"
#include "subsystems/drivetrain.hpp"

#define PB
//define J

#define QUAL_AUTO
//#define MATCH_AUTO

constexpr int8_t FRONT_LEFT_PORT         = 1;
constexpr int8_t MIDDLE_FRONT_LEFT_PORT  = 2;
constexpr int8_t MIDDLE_BACK_LEFT_PORT   = 3;
constexpr int8_t BACK_LEFT_PORT          = 4;
constexpr int8_t FRONT_RIGHT_PORT        = 5;
constexpr int8_t MIDDLE_FRONT_RIGHT_PORT = 6;
constexpr int8_t MIDDLE_BACK_RIGHT_PORT  = 7;
constexpr int8_t BACK_RIGHT_PORT         = 8;

constexpr int8_t INTAKE_1_PORT = 9;
constexpr int8_t INTAKE_2_PORT = 10;

constexpr int8_t CLIMB_1_PORT = 11;
constexpr int8_t CLIMB_2_PORT = 12;
constexpr int8_t CLIMB_3_PORT = 13;
constexpr int8_t CLIMB_4_PORT = 14;

constexpr int8_t HORIZONTAL_POD_PORT = 15;
constexpr int8_t VERTICAL_POD_PORT = 16;
constexpr int8_t GYRO_PORT = 17;

constexpr int ODOM_WHEEL_DIAMETER = 2;
constexpr int HORIZONTAL_WHEEL_DISTANCE = 0;
constexpr int VERTICAL_WHEEL_DISTANCE = 0;

constexpr int TRACK_WIDTH = 12;
constexpr int WHEEL_DIAMETER = 3;
constexpr int DRIVE_RPM = 600;
constexpr int CHASE_POWER = 2;

pros::Controller driver(pros::controller_id_e_t::E_CONTROLLER_MASTER);

//DRIVETRAIN MOTORS

pros::Motor frontLeft(FRONT_LEFT_PORT);
pros::Motor middleFrontLeft(MIDDLE_FRONT_LEFT_PORT);
pros::Motor middleBackLeft(MIDDLE_BACK_LEFT_PORT);
pros::Motor backLeft(BACK_LEFT_PORT);
pros::Motor frontRight(FRONT_RIGHT_PORT);
pros::Motor middleFrontRight(MIDDLE_FRONT_RIGHT_PORT);
pros::Motor middleBackRight(MIDDLE_BACK_RIGHT_PORT);
pros::Motor backRight(BACK_RIGHT_PORT);

pros::Motor_Group leftSide({frontLeft, middleFrontLeft, middleBackLeft, backLeft});
pros::Motor_Group rightSide({frontRight, middleFrontRight, middleBackRight, backRight});

//SENSORS

pros::Rotation horizontalPod(HORIZONTAL_POD_PORT);
pros::Rotation verticalPod(VERTICAL_POD_PORT);
pros::IMU gyro(GYRO_PORT);

//LEMLIB STRUCTURES

lemlib::TrackingWheel horizontalWheel(&horizontalPod, ODOM_WHEEL_DIAMETER, HORIZONTAL_WHEEL_DISTANCE);
lemlib::TrackingWheel verticalWheel(&horizontalPod, ODOM_WHEEL_DIAMETER, VERTICAL_WHEEL_DISTANCE);

lemlib::Drivetrain LLDrivetrain(
    &leftSide,
    &rightSide,
    TRACK_WIDTH,
    WHEEL_DIAMETER,
    DRIVE_RPM,
    CHASE_POWER
);

lemlib::ControllerSettings linearController(
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0
);

lemlib::ControllerSettings angularController(
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0
);

lemlib::OdomSensors sensors(
    &verticalWheel,
    nullptr,
    &horizontalWheel,
    nullptr,
    &gyro
);

lemlib::Chassis chassis(LLDrivetrain, linearController, angularController, sensors);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

void qualPB()
{

}

void matcnPB()
{

}

void qualJ()
{

}

void matchJ()
{
	
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
#if defined(PB)

	#if defined(QUAL_AUTO)
		qualPB();
	#elif defined(MATCH_AUTO)
		matchPB();
	#endif

#elif defined(J)

	#if defined(QUAL_AUTO)
		qualJ();
	#elif defined(MATCH_AUTO)
		matchJ();
	#endif

#endif
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    int l = driver.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int r = driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    chassis.tank(l, r);

//TODO: Control intake
/*
    if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
        ri.spin(-ri.STANDARD_MV);
    }
    else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
        ri.spin(ri.STANDARD_MV);
    }
    else
    {
        ri.spin(0);
    }
*/
}
