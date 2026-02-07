#include "main.h"
#include "liblvgl/llemu.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include "string"
#include "lemlib/api.hpp"

pros::Controller sticks(pros::E_CONTROLLER_MASTER);

/*------------------------- drivetrain + motors ---------------------------------*/
pros::MotorGroup lDrive({-11, 12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rDrive({20, -19 , 18}, pros::MotorGearset::blue);

lemlib::Drivetrain drivetrain(&lDrive, // left motor group
                              &rDrive, // right motor group
                              9.75, // inch track width
                              lemlib::Omniwheel::NEW_325, // wheel type
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

pros::Motor intake(-1, pros::MotorGearset::blue);
pros::Motor indexer(-3, pros::MotorGearset::green);
// SWITCH BACK
pros::Motor roller(2, pros::MotorGearset::green);

pros::adi::DigitalOut park('a');
pros::adi::DigitalOut hood('b');
pros::adi::DigitalOut tongue('c');

/*------------------------ odom + PID config ------------------------------------*/

pros::Rotation horizontal_encoder(-10);
pros::Rotation vertical_encoder(-9);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 2.5);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, .032);

pros::IMU inertial(8);
pros::Optical sorter(7);
pros::Distance dp(6);

std::string mode = "stop";
bool blue = false;

bool pval = false;
int velo = 12000;

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &inertial // inertial sensor
);

lemlib::ControllerSettings lateralPID(7, // proportional gain (kP)
                                    	0, // integral gain (kI)
                                        12, // derivative gain (kD)
                                        3, // anti windup
                                        1, // small error range, in inches
                                        100, // small error range timeout, in milliseconds
                                        3, // large error range, in inches
                                        500, // large error range timeout, in milliseconds
                                        0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angularPID(7, // proportional gain (kP)
                                        0, // integral gain (kI)
                                        40, // derivative gain (kD)
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
void rollers(std::string mode, int v = 12000){
    int intk = 1, idx = 1, rlr = 1;
    if (mode == "store") rlr = 1;
    else if (mode == "low") intk = idx = rlr = -1;
    else if (mode == "mid") rlr = -1;
    else if (mode == "high") rlr = 1;
    else intk = idx = rlr = 0;

    intake.move_voltage(12000*intk);
    indexer.move_voltage(12000*idx);
    roller.move_voltage(v*rlr);

}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    sorter.set_led_pwm(100);
    chassis.calibrate(); // calibrate sensors
    chassis.setPose(0,0,0); 
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
	
            pros::lcd::print(1, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(2, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(3, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(50);
        }
    });

    pros::Task colorSort([&]() {
        bool curblue = blue;
        int frame = 0;
        while (true) {
            if(!pval){
                double hue = sorter.get_hue();
                if (hue < 30 || hue > 170){
                    curblue = (hue > 170);
                }
                if(pros::c::competition_is_autonomous()==false){
                    if(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) mode = "store";
                    else if(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) mode = "low";
                    else if(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) mode = "mid";
                    else if(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) mode = "high";
                    else mode = "stop";

                }
            
                if (mode != "high" && mode != "mid") frame = 0; else ++frame;
                if (frame < 7 && (mode == "high" || mode == "mid")) rollers("low", 9000);
                else if (mode == "high" || mode == "mid") rollers((curblue == curblue) ? mode : (mode == "high" ? "mid" : "high"), velo);
                else rollers(mode);
            }
            pros::delay(30);
        }
    });
  //60 neutral 170 blue 20 red
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
    chassis.setPose(0,0,-90);
    velo = 12000;
    hood.set_value(true);

    chassis.moveToPose(-36, 26,-90, 1900, {.lead=.2, .maxSpeed=100});
    chassis.turnToHeading(180, 900, {}, false);
    tongue.set_value(true);
    pros::delay(400);
    mode="store";
    chassis.moveToPose(-36, -6.7,180, 1800, {.lead = .2, .maxSpeed=100}, false);
    pros::delay(1000);
    chassis.moveToPose(-45, 24,180, 2000, {.forwards=false, .lead = .2, .maxSpeed=100});
    chassis.moveToPose(-45, 84,180, 2000, {.forwards=false, .lead = .2, .maxSpeed=100});
    tongue.set_value(false);
    chassis.moveToPose(-36, 80,0, 2000, {.forwards=false, .lead = .2, .maxSpeed=100}, false);
    mode="high";
    hood.set_value(false);
    pros::delay(4000);
/*
    mode="store";
    hood.set_value(false);
    tongue.set_value(true);
    chassis.moveToPose(-36, 100,0, 2000, {.lead = .2, .maxSpeed=100}, false);
    pros::delay(1000);
    
    chassis.moveToPose(-36, 80,0, 2000, {.forwards=false, .lead = .2, .maxSpeed=100}, false);
    mode="high";
    hood.set_value(false);
    pros::delay(4000);

    chassis.moveToPose(48, 84,90, 2000, {.lead = .2, .maxSpeed=100});
    chassis.turnToHeading(0, 1000);

*/

}
//-21.5, 26.5

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
        velo = 12000;

    bool tval = false;
    while (true){
        int leftY = sticks.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = sticks.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        if(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_L1) || sticks.get_digital(pros::E_CONTROLLER_DIGITAL_L2) ) hood.set_value(false);
        else hood.set_value(true);

        if(sticks.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) && sticks.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) pval = !pval;
        if (pval){
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
            intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

            while (dp.get_distance() > 80){
                intake.move_voltage(-6000);
                roller.move_voltage(-12000);
                indexer.move_voltage(-12000);

                pros::delay(10);
            }
            pros::delay(200);
            intake.move_voltage(0);
            park.set_value(pval);
            pros::delay(50);
        }else {
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
            intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        }

        if(sticks.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))tval = !tval;
        tongue.set_value(tval);
        // delay to save resources
        pros::delay(10);
    }
}