#include "main.h"

#include "subsystems/drivetrain.hpp"

#define PB
//define J

#define QUAL_AUTO
//#define MATCH_AUTO

constexpr int8_t frontLeftPort        = 1;
constexpr int8_t middleFrontLeftPort  = 2;
constexpr int8_t middleBackLeftPort   = 3;
constexpr int8_t backLeftPort         = 4;
constexpr int8_t frontRightPort       = 5;
constexpr int8_t middleFrontRightPort = 6;
constexpr int8_t middleBackRightPort  = 7;
constexpr int8_t backRightPort        = 8;

constexpr int8_t intake1Port = 9;
constexpr int8_t intake2Port = 10;

constexpr int8_t climb1Port = 11;
constexpr int8_t climb2Port = 12;
constexpr int8_t climb3Port = 13;
constexpr int8_t climb4Port = 14;

pros::Controller driver(pros::controller_id_e_t::E_CONTROLLER_MASTER);

pros::Motor frontLeft(frontLeftPort);
pros::Motor middleFrontLeft(middleFrontLeftPort);
pros::Motor middleBackLeft(middleBackLeftPort);
pros::Motor backLeft(backLeftPort);
pros::Motor frontRight(frontRightPort);
pros::Motor middleFrontRight(middleFrontRightPort);
pros::Motor middleBackRight(middleBackRightPort);
pros::Motor backRight(backRightPort);

pros::Motor_Group leftSide({frontLeft, middleFrontLeft, middleBackLeft, backLeft});
pros::Motor_Group rightSide({frontRight, middleFrontRight, middleBackRight, backRight});

TankDrivetrain drivetrain(leftSide, rightSide);

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
    drivetrain.tankControl(driver);

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
