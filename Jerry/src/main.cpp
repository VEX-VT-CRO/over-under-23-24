//JERRY
//Turret robot capable of auto-aim

#include "main.h"
#include "subsystems/tankRobot.hpp"
#include <cstdlib>
#include "lemlib/api.hpp"

pros::Motor leftFront(11, true);
pros::Motor leftMiddle(12, true);
pros::Motor leftBack(13, true);

pros::Motor rightFront(19, false);
pros::Motor rightMiddle(18, false);
pros::Motor rightBack(20, false);

pros::Motor leftside[] = {leftFront, leftMiddle, leftBack};
pros::Motor rightside[] = {rightFront, rightMiddle, rightBack};

//16 - Spool
pros::Motor spoolMotor(16);
pros::MotorGroup spoolGroup({spoolMotor});
Spool spool(spoolGroup, 1);

pros::Motor intake1(17);
pros::MotorGroup riGroup({intake1});
RollerIntake ri(riGroup);

pros::Motor turretMotor1(2);
pros::Motor turretMotor2(15);
pros::IMU turretGyro(5);
pros::Rotation rotated(3);
Turret* turret;

//3 - rotation sensor
pros::ADIDigitalIn catapult_charged('F');
pros::Distance distance_sensor(14);
pros::Motor catapultMotor(4);
Catapult* catapult;

pros::ADIDigitalOut indexerSolenoid('E');

TankDrivetrain drivetrain(leftside, rightside, 3);

//pros::Vision vision_sensor(10);
VisionSensor* vis;

TeamColor team = TeamColor::Blue;


TankRobot* robot;

//LEMLIB (https://lemlib.github.io/LemLib/md_docs_tutorials_2_setting_up_the_chassis->html)
pros::MotorGroup leftSideGroup({leftFront, leftMiddle, leftBack});
pros::MotorGroup rightSideGroup({rightFront, rightMiddle, rightBack});

lemlib::Drivetrain_t LLDrivetrain
{
	&leftSideGroup,
	&rightSideGroup,
	10.875, //Track width (space between groups in inches)
	3.215, //Wheel diameter
	266, //Wheel rpm
};

pros::IMU gyro(1);

pros::ADIEncoder verticalEncoder('A', 'B', true);
pros::Rotation horizontalRotation(7);

//Parameters: ADIEncoder, wheel diameter, distance from center, gear ratio
lemlib::TrackingWheel verticalWheel(&verticalEncoder, 2.9, 0.125, 1);
lemlib::TrackingWheel horizontalWheel(&horizontalRotation, 2.9, 5.5, 1);

lemlib::OdomSensors_t sensors
{
	&verticalWheel,
	nullptr,
	&horizontalWheel,
	nullptr,
	&gyro
};

lemlib::ChassisController_t driveController
{
	30, // kP
    40, // kD
    0.25, // smallErrorRange
    250, // smallErrorTimeout
    1, // largeErrorRange
    1000, // largeErrorTimeout
    10 // slew rate
};

lemlib::ChassisController_t turnController {
    5, // kP
    20, // kD
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

void goTo(float x, float y, int timeout, float maxSpeed = 266.0f, bool reversed = false, bool log = false)
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
lemlib::Pose target = lemlib::Pose(48,0,0);

void initialize() {
	chassis = new lemlib::Chassis(LLDrivetrain, driveController, turnController, sensors);
	chassis->calibrate();
	chassis->setPose(-23.5,-60,90);
	turret = new Turret(turretMotor1, turretMotor2, rotated, turretGyro, *chassis, target);
	//vis = new VisionSensor(vision_sensor);
	catapult = new Catapult(&catapultMotor, rotated);

	robot = new TankRobot(drivetrain, ri, nullptr, turret, nullptr, catapult, &spool, team);
	pros::lcd::initialize();
	
	pros::Task screenTask1(screen);
	// pros::Task screenTask2(autoaim);
	turret->turnAngle(180);
	pros::delay(2000);
	turretGyro.reset(true);

	turret->rotateback();
	while(!catapult->shoot_ready)
	{
		catapult->charge_state = true;
		pros::delay(20);
		catapult->charge();
	}
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

void intakeBall(bool pipe = false, float speed = ri.STANDARD_MV)
{
	int extendTime = 1000;
	int sweepTime = 400;

	ri.spin(ri.STANDARD_MV);
	if(pipe)
	{
		spool.moveTo(EXTENDED);
		pros::delay(extendTime);
		spool.moveTo(RETRACTED);
		pros::delay(extendTime);
	}
	spool.moveTo(SWEEP);
	pros::delay(sweepTime);
	spool.moveTo(RETRACTED);
	pros::delay(sweepTime * 2);
	ri.spin(0);
}

void shootBall(bool charge = true, bool firstCharge = false)
{
	catapult->charge();
	turret->rotateback();
	int timer = pros::millis();
	while(firstCharge && pros::millis() - timer < 1500)
	{
		catapult->charge();
		pros::delay(10);
	}
	
	//turret->updatePosition();
	//pros::delay(1500);
	catapult->shoot();
	pros::delay(200);
	catapult->charge();
	turret->rotateback();

	timer = pros::millis();
	while(charge && pros::millis() - timer < 1500)
	{
		catapult->charge();
		pros::delay(10);
	}
}

void chargeCatapult()
{
	catapult->charge();
	turret->rotateback();
	int timer = pros::millis();
	while(pros::millis() - timer < 1500)
	{
		catapult->charge();
		pros::delay(10);
	}
}

//For use in a qualifier match with Tom
void tomQual()
{	
	leftSideGroup.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
	rightSideGroup.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
	chassis->setPose(-23.5, -60, 90);
	shootBall();
	ri.spin(ri.STANDARD_MV);
	goTo(20, -60, 750);
	intakeBall(false);
	shootBall(false);
	catapult->half();
	goTo(36, -60, 5000);
	goTo(48, -48, 1250);
	ri.spin(ri.STANDARD_MV);
	spool.moveTo(EXTENDED);
	goTo(55, -55, 1000);
	ri.spin(3000);
	chassis->moveTo(48, -48, 750);
	spool.moveTo(RETRACTED);
	goTo(60, -36, 1250);
	chassis->turnTo(60, -12, 1500);
	ri.spin(-ri.STANDARD_MV);
	chassis->moveTo(60, -40, 750);
	ri.spin(0);
	chassis->turnTo(60, -32, 1250, true);
	chassis->moveTo(60, -0, 1000);
	goTo(60, -36, 750);
	goTo(36, -60, 1250);
	goTo(-36, -60, 5000);
	goTo(-48, -48, 1250);
	chargeCatapult();
	ri.spin(ri.STANDARD_MV);
	goTo(-54, -54, 1000);
	intakeBall(true);
	shootBall();
	chassis->moveTo(-48, -48, 750);
	goTo(-36, -56, 1250);
	goTo(0, -60, 1500);
	goTo(0, -48, 20000);
	leftSideGroup.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	rightSideGroup.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
}

void elimQual()
{
	chassis->setPose(-54, -54, 90);
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
	intakeBall(true);
	shootBall();
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
	elimQual();

	//TEST AUTON
	//odom->setPosition({16, 30.5}); //START
	//odom->setAngle(0);
	//robot->goTo({36, 30.5}, 15000); //
	//chassis->moveTo(36, -36, 5000, 50);
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


	//chassis->setPose(0, 0, 0);
	//chassis->moveTo(0, 10, 3000);
	//chassis->turnTo(5, 8.66, 5000);
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
		robot->pollController(true);
		if (catapult->charge_state) {
			catapult->charge();
		}
		if (catapult->shoot_state){
			catapult->shoot();
		}
		turret->checkRotation();
		pros::delay(10);
	}
}