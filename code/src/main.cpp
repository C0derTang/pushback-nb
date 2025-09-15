#include "main.h"
#include "string"
#include "lemlib/api.hpp"

pros::Controller sticks(pros::E_CONTROLLER_MASTER);

/*------------------------- drivetrain + motors ---------------------------------*/
pros::MotorGroup lDrive({-8, -9, -10}, pros::MotorGearset::blue);
pros::MotorGroup rDrive({1, 2, 3}, pros::MotorGearset::blue);

lemlib::Drivetrain drivetrain(&lDrive, // left motor group
                              &rDrive, // right motor group
                              9.567, // inch track width
                              lemlib::Omniwheel::NEW_325, // wheel type
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

pros::Motor intake(11, pros::MotorGearset::blue);
pros::Motor indexer(19, pros::MotorGearset::green);
pros::Motor roller(20, pros::MotorGearset::green);


/*------------------------ odom + PID config ------------------------------------*/

/* example things
pros::Rotation horizontal_encoder(20);
pros::adi::Encoder vertical_encoder('C', 'D', true);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);
*/

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr // inertial sensor
);

lemlib::ControllerSettings lateralPID(10, // proportional gain (kP)
                                    	0, // integral gain (kI)
                                        3, // derivative gain (kD)
                                        3, // anti windup
                                        1, // small error range, in inches
                                        100, // small error range timeout, in milliseconds
                                        3, // large error range, in inches
                                        500, // large error range timeout, in milliseconds
                                        20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angularPID(2, // proportional gain (kP)
                                        0, // integral gain (kI)
                                        10, // derivative gain (kD)
                                        3, // anti windup
                                        1, // small error range, in degrees
                                        100, // small error range timeout, in milliseconds
                                        3, // large error range, in degrees
                                        500, // large error range timeout, in milliseconds
                                        0 // maximum acceleration (slew)
);


lemlib::ExpoDriveCurve throttle(3, // joystick deadband out of 127
                                10, // minimum output where drivetrain will move out of 127
                                1.019 // expo curve gain
);

lemlib::ExpoDriveCurve steer(3, // joystick deadband out of 127
                             10, // minimum output where drivetrain will move out of 127
                             1.019 // expo curve gain
);


// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateralPID, // lateral PID settings
                        angularPID, // angular PID settings
                        sensors, // odometry sensors
						&throttle, 
                        &steer
);

/*-------------Custom motor functions-----------------*/
void rollers(std::string mode){
    int intk = -1, idx = -1, rlr = 0;
    if (mode == "store") idx = 1;
    else if (mode == "low") intk = 1;
    else if (mode == "mid") rlr = -1;
    else if (mode == "high") rlr = 1;
    else intk = idx = 0;

    intake.move_voltage(12000*intk);
    indexer.move_voltage(12000*idx);
    roller.move_voltage(12000*rlr);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
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
    std::string mode = "stop";
    while (true){
        int leftY = sticks.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = sticks.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.curvature(leftY, rightX);

        if(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) mode = "store";
        else if(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) mode = "low";
        else if(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) mode = "mid";
        else if(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) mode = "high";
        else mode = "stop";
        rollers(mode);

        // delay to save resources
        pros::delay(10);
    }
}