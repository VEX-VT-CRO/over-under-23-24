#include <main.h>
#include <subsystems/tankRobot.hpp>
#include <cstdlib>

pros::Motor leftFront(1, true);
pros::Motor leftMiddle(2, false);
pros::Motor leftBack(3, true);

pros::Motor rightFront(4, false);
pros::Motor rightMiddle(5, true);
pros::Motor rightBack(6, false);

pros::Motor leftside[] = {leftFront, leftMiddle, leftBack};
pros::Motor rightside[] = {rightFront, rightMiddle, rightBack};

pros::IMU gyro(7);
pros::IMU accel(8);

pros::Motor intake(20);
RollerIntake ri(intake);

TankDrivetrain* drivetrain;
TankRobot* robot;
Odometry* odom;
TeamColor team = TeamColor::Blue;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	drivetrain = new TankDrivetrain(leftside, rightside, 3);
	robot = new TankRobot(*drivetrain, ri, odom, team, NULL, PIDConstants{0, 0, 0}, PIDConstants{0, 0, 0});
	pros::lcd::initialize();
	pros::delay(3500);
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
void autonomous() {}

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

	while (true) {
		robot -> pollController(false);
		
		pros::delay(20);
	}
}
