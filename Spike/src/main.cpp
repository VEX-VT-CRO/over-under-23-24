//Spike
//Pass through robot

#include "main.h"
#include "subsystems/tankRobot.hpp"
#include <cstdlib>
#include "lemlib/api.hpp"

pros::Motor leftFront(2, true);
pros::Motor leftMiddle(1, true);
pros::Motor leftBack(11, true);

pros::Motor rightFront(6, false);
pros::Motor rightMiddle(10, false);
pros::Motor rightBack(20, false);

pros::Motor leftside[] = {leftFront, leftMiddle, leftBack};
pros::Motor rightside[] = {rightFront, rightMiddle, rightBack};

pros::Motor conveyor1(3);
pros::Motor conveyor2(8, true);
pros::MotorGroup convGroup({conveyor1, conveyor2});
Conveyor conveyor(convGroup);

pros::Motor intake(7);
pros::MotorGroup riGroup({intake});
RollerIntake ri(riGroup);

pros::ADIDigitalOut indexerSolenoid('F');
Indexer i(indexerSolenoid);

pros::ADIDigitalOut hoodSolenoid('H');
Hood hood(hoodSolenoid);

TankDrivetrain drivetrain(leftside, rightside, 3);

TeamColor team = TeamColor::Blue;


TankRobot* robot;

//LEMLIB (https://lemlib.github.io/LemLib/md_docs_tutorials_2_setting_up_the_chassis->html)
pros::MotorGroup leftSideGroup({leftFront, leftMiddle, leftBack});
pros::MotorGroup rightSideGroup({rightFront, rightMiddle, rightBack});

lemlib::Drivetrain_t LLDrivetrain
{
	&leftSideGroup,
	&rightSideGroup,
	11, //Track width (space between groups in inches)
	3.25, //Wheel diameter
	267, //Wheel rpm
};

pros::IMU gyro(18);

//pros::ADIEncoder verticalEncoder('A', 'B');
//pros::ADIEncoder horizontalEncoder('C', 'D');

//Parameters: ADIEncoder, wheel diameter, distance from center, gear ratio
//lemlib::TrackingWheel leftWheels(&leftSideGroup, 2.72, 0, 1);
//lemlib::TrackingWheel rightWheels(&rightSideGroup, 2.75, 6, 1);

lemlib::OdomSensors_t sensors
{
	nullptr,
	nullptr,
	nullptr,
	nullptr,
	&gyro
};

lemlib::ChassisController_t driveController
{
	8, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};

lemlib::ChassisController_t turnController {
    4, // kP
    40, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};

lemlib::Chassis* chassis;

//Combine turnTo and moveTo into a single function
void goTo(float x, float y, int timeout, float maxDriveSpeed, float maxTurnSpeed, bool reversed = false, bool log = false)
{
	chassis->turnTo(x, y, timeout, reversed, maxTurnSpeed, log);
	chassis->moveTo(x, y, timeout, maxDriveSpeed, log);
}

void goTo(float x, float y, int timeout, float maxSpeed = 127.0f, bool reversed = false, bool log = false)
{
	chassis->turnTo(x, y, timeout, reversed, maxSpeed, log);
	chassis->moveTo(x, y, timeout, maxSpeed, log);
}

void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis->getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	robot = new TankRobot(drivetrain, ri, &i, &hood, &conveyor, nullptr, team);
	//chassis = new lemlib::Chassis(LLDrivetrain, driveController, turnController, sensors);

	pros::lcd::initialize();

	//chassis->calibrate();
	//pros::Task screenTask(screen); // create a task to print the position to the screen
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

void jerryQual()
{
	chassis->setPose(-12, -60, 90);
	ri.spin(ri.STANDARD_MV);
	goTo(12, -60, 5000);
	chassis->turnTo(12, -48, 5000, false);
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
	//Match Auto (Marco)
	
	// chassis->setPose({-32, -60, 0});
	// chassis->moveTo(-32, -50, 10000);
	
	//wait for TOM
	leftSideGroup.move_voltage(0);
	rightSideGroup.move_voltage(0);
	pros::delay(35000);

	//First curve
	leftSideGroup.move_voltage(-8800);
	rightSideGroup.move_voltage(-5000);
	pros::delay(1700);
	leftSideGroup.move_voltage(0);
	rightSideGroup.move_voltage(0);
	pros::delay(200);

	//Straight under the bar
	leftSideGroup.move_voltage(-9000);
	rightSideGroup.move_voltage(-9300);
	i.indexDisc();
	pros::delay(1750);

	
	//2nd curve
	leftSideGroup.move_voltage(-8000);
	rightSideGroup.move_voltage(-6175);
	pros::delay(1250);
	i.indexDisc();

	//pause
	leftSideGroup.move_voltage(0);
	rightSideGroup.move_voltage(0);
	pros::delay(100);

	//face goal
	leftSideGroup.move_voltage(-6000);
	rightSideGroup.move_voltage(6000);
	pros::delay(375);

	leftSideGroup.move_voltage(0);
	rightSideGroup.move_voltage(0);
	pros::delay(200);

	//score
	leftSideGroup.move_voltage(-12000);
	rightSideGroup.move_voltage(-12000);
	pros::delay(700);
	
	leftSideGroup.move_voltage(0);
	rightSideGroup.move_voltage(0);
	pros::delay(200);

	leftSideGroup.move_voltage(12000);
	rightSideGroup.move_voltage(10000);
	pros::delay(300);

	leftSideGroup.move_voltage(0);
	rightSideGroup.move_voltage(0);
	pros::delay(750);

	leftSideGroup.move_voltage(-12000);
	rightSideGroup.move_voltage(-12000);
	pros::delay(550);
	
	leftSideGroup.move_voltage(0);
	rightSideGroup.move_voltage(0);
	pros::delay(100);

	leftSideGroup.move_voltage(10000);
	rightSideGroup.move_voltage(12000);
	pros::delay(300);

	leftSideGroup.move_voltage(0);
	rightSideGroup.move_voltage(0);
	pros::delay(1000);

	//TEST AUTON
	//odom->setPosition({16, 30.5}); //START
	//odom->setAngle(0);
	//chassis->setPose(-12, -60, -90);
	//robot->goTo({36, 30.5}, 15000); //

	// leftSideGroup.move_voltage(-10000);
	// rightSideGroup.move_voltage(-10000);
	// pros::delay(1800);
	// i.indexDisc();
	// leftSideGroup.move_voltage(-10000);
	// rightSideGroup.move_voltage(-7000);
	// pros::delay(2000);
	// leftSideGroup.move_voltage(11000);
	// rightSideGroup.move_voltage(11000);
	// pros::delay(500);
	// leftSideGroup.move_voltage(-11000);
	// rightSideGroup.move_voltage(-11000);
	// pros::delay(500);
	// leftSideGroup.move_voltage(11000);
	// rightSideGroup.move_voltage(11000);
	// pros::delay(500);
	// leftSideGroup.move_voltage(-11000);
	// rightSideGroup.move_voltage(-11000);
	// pros::delay(500);
	// leftSideGroup.move_voltage(11000);
	// rightSideGroup.move_voltage(11000);
	// pros::delay(500);
	// leftSideGroup.move_voltage(0);
	// rightSideGroup.move_voltage(0);


	//goTo(36, -36, 5000);
	//robot->goTo({47, 59.75}, 15000); //First triball
	//goTo(12, 0, 5000);
	//robot->goTo({63.5, 59.75}, 15000); //Second triball
	//goTo(63.5, 59.75, 15000);
	//robot->goTo({28.5, 14}, 15000); //Left of bar
	//goTo(28.5, 14, 15000);
	//robot->goTo({99.5, 14}, 15000); //Right of bar
	//goTo(99.5, 14, 15000);
	//robot->goTo({108, 29}, 15000); //Get ready for the turn
	//goTo(108, 29, 15000);
	//robot->goTo({94, 47}, 15000); //About to go to third triball
	//goTo(94, 47, 15000);
	//robot->goTo({77.5, 47}, 15000); //Third triball
	//goTo(77.5, 47, 15000);
	//robot->goTo({110.5, 58.75}, 15000); //Push it in
	//goTo(110.5, 58.75, 15000);
	//robot->goTo({75.5, 23.5}, 15000); //Ram the climb post
	//goTo(75.5, 23.5, 15000);

	//TESTING PID
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

		pros::delay(10);
	}
}