#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "liblvgl/lvgl.h"

int red, green;

pros::Controller controller(pros::E_CONTROLLER_MASTER); // sets up controller

pros::Motor right1(1);
pros::Motor right2(2);
pros::Motor right3(-3);
pros::Motor left1(-4);
pros::Motor left2(5);
pros::Motor left3(-6);

pros::MotorGroup rightDrive({1, 2, -3}, pros::MotorGearset::blue); // creates the right drivetrain motor group with forwards ports 1 & 3 and backwards port 2
pros::MotorGroup leftDrive({-4, 5, -6}, pros::MotorGearset::blue); // creates the left drivetrain motor group with forwards port 5 and backwards ports 4 & 6

pros::Motor roller1(7);
pros::Motor roller2(8);
pros::Motor roller3(9);
pros::Motor roller4(11);

pros::IMU imu(10);

pros::Rotation verticalRotationSensor(-12);

pros::adi::DigitalOut toungeMech('H');

lemlib::TrackingWheel verticalTrackingWheel(&verticalRotationSensor, lemlib::Omniwheel::NEW_275, 0);

// drivetrain settingsMore actions
lemlib::Drivetrain drivetrain(&leftDrive, // left motor group
							  &rightDrive, // right motor group
							  12.5, // 12.5 inch track width (NEEDS TO BE MEASURED)
							  lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
							  480, // drivetrain rpm is 480 rpm
							  2 // horizontal drift is 2 (for now)
);


// odometry settings
lemlib::OdomSensors sensors(&verticalTrackingWheel, // vertical tracking wheel 1, set to null (temporary)
  							nullptr, // vertical tracking wheel 2, set to null
  							nullptr, // horizontal tracking wheel 1, set to null (temporary hopefully)
  							nullptr, // horizontal tracking wheel 2, set to null
  							&imu // inertial sensor, set to null (temporary)
);


// lateral PID controller
lemlib::ControllerSettings lateral_controller(15, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              5, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              100, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
						lateral_controller, // lateral PID settings
						angular_controller, // angular PID settings
						sensors // odometry sensors
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    chassis.calibrate(); // calibrate sensors
    pros::lcd::initialize();
}

/**
 * Displays data for sensors, battery, and motors.
 * 
 * Uses the LCD display on the brain to display the data.
 
void dataDisplay () {
    while(true){
        int rightPower1 = right1.get_power(); // outputs the power from the first motor in the right motor group in watts
        int rightPower2 = right2.get_power(); // outputs the power from the second motor in the right motor group in watts
        int rightPower3 = right3.get_power(); // outputs the power from the third motor in the right motor group in watts
    
        int leftPower1 = left1.get_power(); // outputs the power from the first motor in the left motor group in watts
        int leftPower2 = left2.get_power(); // outputs the power from the second motor in the left motor group in watts
        int leftPower3 = left3.get_power(); // outputs the power from the third motor in the left motor group in watts
    
        int batteryPercent = pros::battery::get_capacity();

        red = 510 - (batteryPercent*5.1);
        green = batteryPercent*5.1;

        pros::screen::erase();
        
        pros::screen::set_pen(RGB2COLOR(red, green, 20));
        pros::screen::print(TEXT_MEDIUM, 1, "Data n' Stuff:");
        pros::screen::print(TEXT_MEDIUM, 2, "Right Motor 1 (Port 1) Power: %dW", rightPower1);
        pros::screen::print(TEXT_MEDIUM, 3, "Right Motor 2 (Port 2) Power: %dW", rightPower2);
        pros::screen::print(TEXT_MEDIUM, 4, "Right Motor 3 (Port 3) Power: %dW", rightPower3);
        pros::screen::print(TEXT_MEDIUM, 5, "Left Motor 1 (Port 4) Power: %dW", leftPower1);
        pros::screen::print(TEXT_MEDIUM, 6, "Left Motor 2 (Port 5) Power: %dW", leftPower2);
        pros::screen::print(TEXT_MEDIUM, 7, "Left Motor 3 (Port 6) Power: %dW", leftPower3);
        pros::screen::print(TEXT_MEDIUM, 8, "Battery Percentage: %d %", batteryPercent);

        pros::delay(50);
    }
}

void logTask() {
    int time = 0;

    while (true) {
        int rightPower1 = right1.get_power(); // outputs the power from the first motor in the right motor group in watts
        int rightPower2 = right2.get_power(); // outputs the power from the second motor in the right motor group in watts
        int rightPower3 = right3.get_power(); // outputs the power from the third motor in the right motor group in watts
    
        int leftPower1 = left1.get_power(); // outputs the power from the first motor in the left motor group in watts
        int leftPower2 = left2.get_power(); // outputs the power from the second motor in the left motor group in watts
        int leftPower3 = left3.get_power(); // outputs the power from the third motor in the left motor group in watts
        
        std::string logMessage = std::to_string(time) +
            "," + std::to_string(rightPower1) + 
            "," + std::to_string(rightPower2) + 
            "," + std::to_string(rightPower3) + 
            "," + std::to_string(leftPower1) + 
            "," + std::to_string(leftPower2) + 
            "," + std::to_string(leftPower3) + "\n";

        printf(logMessage.c_str());

        time += 10;

        pros::delay(10);
    }
}
*/

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    
}

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
    

    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 48, 9999999, {.maxSpeed = 70});

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
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    int time = 0;

	while (true) {

        int rightPower1 = right1.get_power(); // outputs the power from the first motor in the right motor group in watts
        int rightPower2 = right2.get_power(); // outputs the power from the second motor in the right motor group in watts
        int rightPower3 = right3.get_power(); // outputs the power from the third motor in the right motor group in watts
    
        int leftPower1 = left1.get_power(); // outputs the power from the first motor in the left motor group in watts
        int leftPower2 = left2.get_power(); // outputs the power from the second motor in the left motor group in watts
        int leftPower3 = left3.get_power(); // outputs the power from the third motor in the left motor group in watts
        
        std::string logMessage = std::to_string(time) +
            "," + std::to_string(rightPower1) + 
            "," + std::to_string(rightPower2) + 
            "," + std::to_string(rightPower3) + 
            "," + std::to_string(leftPower1) + 
            "," + std::to_string(leftPower2) + 
            "," + std::to_string(leftPower3) + "\n";

        printf(logMessage.c_str());

        time += 25;

        int batteryPercent = pros::battery::get_capacity();

        red = 510 - (batteryPercent*5.1);
        green = (batteryPercent*5.1);

        pros::screen::erase();
        
        pros::screen::set_pen(RGB2COLOR(red, green, 20));
        pros::screen::print(TEXT_MEDIUM, 1, "Data n' Stuff:");
        pros::screen::print(TEXT_MEDIUM, 2, "Right Motor 1 (Port 1) Power: %dW", rightPower1);
        pros::screen::print(TEXT_MEDIUM, 3, "Right Motor 2 (Port 2) Power: %dW", rightPower2);
        pros::screen::print(TEXT_MEDIUM, 4, "Right Motor 3 (Port 3) Power: %dW", rightPower3);
        pros::screen::print(TEXT_MEDIUM, 5, "Left Motor 1 (Port 4) Power: %dW", leftPower1);
        pros::screen::print(TEXT_MEDIUM, 6, "Left Motor 2 (Port 5) Power: %dW", leftPower2);
        pros::screen::print(TEXT_MEDIUM, 7, "Left Motor 3 (Port 6) Power: %dW", leftPower3);
        pros::screen::print(TEXT_MEDIUM, 8, "Battery Percentage: %d %", batteryPercent);

		int tankLeft = controller.get_analog(ANALOG_LEFT_Y); // the left analog stick controls the left side of the drive train
        int tankRight = controller.get_analog(ANALOG_RIGHT_Y); // the right analog stick controls the left side of the drive train        
        chassis.tank(tankLeft, tankRight); // creates a tank drive scheme
        /*
        if(controller.get_digital(DIGITAL_R1)){
            roller1.move(127);
            roller3.move(-127);
        }
        else if(controller.get_digital(DIGITAL_R2)){
            roller1.move(-127);
            roller3.move(127);
        }
        else{
            roller1.move(0);
            roller3.move(0);
        }
        */

        if(controller.get_digital(DIGITAL_R1)){
            roller1.move(127);
            roller2.move(-127);
        } 
        else if(controller.get_digital(DIGITAL_R2)){
            roller1.move(127);
            roller2.move(127);
            roller3.move(-127);
        }
        else if(controller.get_digital(DIGITAL_L1)){
            roller1.move(-127);
            roller2.move(127);
        }
        else if(controller.get_digital(DIGITAL_L2)){
            roller1.move(127);
            roller2.move(127);
            roller3.move(127);
            roller4.move(127);
        }
        else{
            roller1.move(0);
            roller2.move(0);
            roller3.move(0);
            roller4.move(0);
        }
        
        if(controller.get_digital(DIGITAL_B) || controller.get_digital(DIGITAL_Y)){
            toungeMech.set_value(true);
        }
        else if(controller.get_digital(DIGITAL_DOWN) || controller.get_digital(DIGITAL_RIGHT)){
            toungeMech.set_value(false);
        }

		pros::delay(25); // wait 25ms to save resources
	}
}