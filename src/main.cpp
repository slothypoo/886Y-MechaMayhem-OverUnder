#include "main.h"
#include "lemlib/api.hpp"
#include "pros/rotation.hpp"

//drive motors

pros::Motor lF(-7, pros::E_MOTOR_GEARSET_06);
pros::Motor rF(6, pros::E_MOTOR_GEARSET_06);
pros::Motor lB(-17, pros::E_MOTOR_GEARSET_06);
pros::Motor rB(3, pros::E_MOTOR_GEARSET_06);
pros::Motor lM(-8, pros::E_MOTOR_GEARSET_06);
pros::Motor rM(5, pros::E_MOTOR_GEARSET_06);

//other motors
pros::Motor intake(4, pros::E_MOTOR_GEARSET_06);
pros::Motor hang(20, pros::E_MOTOR_GEARSET_06);

//sensors
pros::Imu imu(11);  
pros::Rotation hangRot(19);

//pistons
pros::ADIDigitalOut frontWings(1);	
pros::ADIDigitalOut backWings(2);
pros::ADIDigitalOut ratchet(4);

//controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({lF, lM, lB}); // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group
  
// tracking wheels
pros::Rotation vertEnc(15, true);
pros::Rotation horEnc(13, true);

lemlib::TrackingWheel vertical(&vertEnc, lemlib::Omniwheel::NEW_275, -3.7);
lemlib::TrackingWheel horizontal(&horEnc, lemlib::Omniwheel::NEW_275, -3.7);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 3.25" omnis
                              450, // drivetrain rpm is 360s
                              2 // chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
// note that in this example we use internal motor encoders (IMEs), so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    
}

/**
 * Runs in driver control
 */
void opcontrol() {

	bool tog = false;
	bool wingState = false;
    bool bwingState = false;
	bool blockState = false;
    bool ratchetState = false;
	

	while(true){
		
 
		leftArcade(); // Alan Preferred
	
	
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Flywheel/Intake

			if(master.get_digital(DIGITAL_L1) == 1){
				intake = -127;
		    }

			else if(master.get_digital(DIGITAL_L2) == 1){
				intake = 127;
		    }

			else{
				intake = 0;
			}
		


		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Piston wings



			if(master.get_digital_new_press(DIGITAL_Y) == 1){
				wingState = !wingState;
		    }
			
			if(wingState == false){
				frontWings.set_value(0);
			}
			else if(wingState == true){
				frontWings.set_value(1);
			}



            if(master.get_digital_new_press(DIGITAL_B) == 1){
				bwingState = !bwingState;
		    }
			
			if(bwingState == false){
				backWings.set_value(0);
			}
			else if(bwingState == true){
				backWings.set_value(1);
			}


        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Ratchet

            if(master.get_digital_new_press(DIGITAL_RIGHT) == 1){
				ratchetState = !ratchetState;
		    }
			
			if(bwingState == false){
				ratchet.set_value(0);
			}
			else if(bwingState == true){
				ratchet.set_value(1);
			}




		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~hang

            if(master.get_digital(DIGITAL_R1) == 1){
				hang = -127;
		    }
		    else if(master.get_digital(DIGITAL_R2) == 1){
				hang = 127;
		    }
		    else{
				hang = 0;
			}
		
		
		
				

			

		pros::delay(10);

	}

}