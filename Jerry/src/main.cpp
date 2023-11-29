#include "main.h"
#include "subsystems/tankRobot.hpp"
#include <cstdlib>

pros::Motor leftFront(1, true);
pros::Motor leftMiddle(2, true);
pros::Motor leftBack(3, true);

pros::Motor rightFront(11, false);
pros::Motor rightMiddle(12, false);
pros::Motor rightBack(13, false);

pros::Motor leftside[] = {leftFront, leftMiddle, leftBack};
pros::Motor rightside[] = {rightFront, rightMiddle, rightBack};

pros::Motor intake(7);
RollerIntake ri(intake);

pros::Motor turretMotor1(8);
pros::Motor turretMotor2(9);
pros::IMU turretGyro(17);
Turret* turret;

pros::ADIDigitalIn catapult_charged(18);
pros::Motor catapultMotor(15);
Catapult* catapult;

pros::ADIDigitalOut indexerSolenoid('E');
Indexer i(indexerSolenoid);

TankDrivetrain drivetrain(leftside, rightside, 3);

Odometry* odom;

pros::Vision vision_sensor(10);
VisionSensor* vis;

TeamColor team = TeamColor::Blue;

PIDConstants forwardDrive = {12, 0, 5};
PIDConstants inPlaceTurn = {0.01, 0, 0};

TankRobot* robot;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	turret = new Turret(turretMotor1, turretMotor2, turretGyro, {0, 0, 0});
	odom = new Odometry(18, 'A', 'B', 'C', 'D');
	catapult = new Catapult(&catapultMotor,odom, &catapult_charged);
	vis = new VisionSensor(vision_sensor);

	robot = new TankRobot(drivetrain, ri, i, turret, vis, odom, catapult, team, forwardDrive, inPlaceTurn);

	pros::lcd::initialize();
	pros::delay(3500);
	odom->setPosition({0.0, 0.0});
	odom->setAngle(0);
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
void autonomous() {
	//TEST AUTON
	odom->setPosition({16, 30.5}); //START
	odom->setAngle(0);
	robot->goTo({36, 30.5}, 15000); //
	robot->goTo({47, 59.75}, 15000); //First triball
	//robot->goTo({63.5, 59.75}, 15000); //Second triball
	//robot->goTo({28.5, 14}, 15000); //Left of bar
	//robot->goTo({99.5, 14}, 15000); //Right of bar
	//robot->goTo({108, 29}, 15000); //Get ready for the turn
	//robot->goTo({94, 47}, 15000); //About to go to third triball
	//robot->goTo({77.5, 47}, 15000); //Third triball
	//robot->goTo({110.5, 58.75}, 15000); //Push it in
	//robot->goTo({75.5, 23.5}, 15000); //Ram the climb post

	/*while(odom->getPosition().y < 70.5)
	{
		leftFront.move_voltage(2250);
		leftMiddle.move_voltage(2250);
		leftBack.move_voltage(2250);
		rightFront.move_voltage(3000);
		rightMiddle.move_voltage(3000);
		rightBack.move_voltage(3000);
		
		pros::delay(10);
		odom->update();
	}

	drivetrain.drive(0);

	pros::delay(1500);

	while(odom->getPosition().y > 0.0)
	{
		leftFront.move_voltage(-2700);
		leftMiddle.move_voltage(-2700);
		leftBack.move_voltage(-2700);
		rightFront.move_voltage(-3000);
		rightMiddle.move_voltage(-3000);
		rightBack.move_voltage(-3000);
		
		pros::delay(10);
		odom->update();
	}*/
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

	while (true) {
		robot->pollController(false);
		
		pros::delay(20);
	}
}
