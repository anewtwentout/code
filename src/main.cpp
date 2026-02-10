#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
//variables
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
//pnematics
pros::adi::DigitalOut scraper ('D', false);
pros::adi::DigitalOut intakeLift ('A', false);
pros::adi::DigitalOut wings ('B', false);
pros::adi::DigitalOut redirect ('C', true);
// motor groups
pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({14, 15, 16}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)
pros::Motor firstStageMotor(1, pros::MotorGearset::blue);
pros::Motor secondStageMotor(2, pros::MotorGearset::blue);

// Inertial Sensor on port 10
pros::Imu imu(10);
/*
pros::Distance distance(1);
chassis.setPos(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
*/
// tracking wheels
//horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
// pros::Rotation horizontalEnc(0);
// //vertical tracking wheel encoder. Rotation sensor, port 11, reversed
// pros::Rotation verticalEnc(-11);
//horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)

//vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
// lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller

lemlib::ControllerSettings linearController(8, // proportional gain (kP)
                                            0, // integral gain (kI)
                                           54.1, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller

lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0,// integral gain (kI)
                                             12.5, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(5, // joystick deadband out of 127
                                     5, // minimum output where drivetrain will move out of 127
                                     1.012 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  5, // minimum output where drivetrain will move out of 127
                                  1.012 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
float codeVersion = 123; // just for debugging purposes cause sometimes the code doesnt upload.
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    chassis.setPose(0,0,0);
    wings.set_value(false);
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(1, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(2, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(3, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(4, "Version: %f", codeVersion); // heading
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

bool autonomousRunning = false;
int autonomousValue = 2; // 0 = Skills, 1 = R7+Wings, 2 = L4+3, 3 = R7+Blocks, 4 = L7
int amountOfAutonomousCodes = 6;
void competition_initialize() {
  pros::Task autonomousTask([&]() {
  while(true)
  {
  if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
    autonomousValue = (autonomousValue+1)%amountOfAutonomousCodes;
  }
  if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
    autonomousValue = (autonomousValue-1)%amountOfAutonomousCodes;
  }
  
  switch(autonomousValue){
    case 0:
      pros::lcd::print(0, "Autonomous: %s", "Skill");
      break;
    case 1:
      pros::lcd::print(0, "Autonomous: %s", "Right");
      break;
    case 2:
      pros::lcd::print(0, "Autonomous: %s", "Left4+3");
      break;
  	case 3:
	    pros::lcd::print(0, "Autonomous: %s", "RightBlocks");
      break;
      case 4:
      pros::lcd::print(0, "Autonomous: %s", "Lef7");
      break;
      case 5:
      pros::lcd::print(0, "Autonomous: %s", "SAWP");
      break;
  }
  pros::delay(50);
  }
});

}


// // get a path used for pure pursuit
// // this needs to be put outside a function
// ASSET(part1_txt);
// void right2(){ //PATH.JERRYIO
//   /*chassis.moveTo(0, 0, 5000);YAY
// chassis.moveTo(0, 5, 5000); YAY
// chassis.moveTo(3.042, 14.227, 5000); YAY
// chassis.moveTo(5.87, 23.252, 5000); YAY
// chassis.moveTo(31.869, 1.834, 5000); YAY
// chassis.moveTo(31.869, -10.137, 5000); NEEDS TO E ADJUSTED BY SCRAPER LENGTH
// chassis.moveTo(31.869, -20.644, 5000); YAY
// chassis.moveTo(31.869, 15.526, 5000); YAY
// chassis.moveTo(31.869, 13.026, 5000);
// chassis.moveTo(41.587, 13.026, 5000); 
// chassis.moveTo(41.587, 39.526, 5000);
// */
//   wings.set_value(true);
//   scraper.set_value(false);
//   hood.set_value(false);
//   chassis.setPose(-1.75, 0, 0);
//   //Moves ahead a little
//   chassis.moveToPoint(0,5,200,{}, false);
//   intakeMotor.move(127);
//   midStageMotor.move(127);
//   finalStageMotor.move(127);
//   //Move right Before The First Blocks
//   chassis.turnToPoint(3.042,14.227,300, {}, false);
//   chassis.moveToPoint(3.042,14.227, 500, {.minSpeed=80, .earlyExitRange = 3}, false);
//   //Move To First Blocks
//   scraper.set_value(true);
//   chassis.moveToPoint(5.87, 23.252, 400, {}, false);
//   //In Between
//   chassis.turnToPoint(31.869, 1.834, 600, {}, false);
//   chassis.moveToPoint(31.869,1.834, 1250, {}, false);
//   //Right Before Loader
//   chassis.turnToPoint(31.869,-10.137+2.5+6,500,{},false);
//   chassis.moveToPoint(31.869,-10.137+2.5+6,250,{.minSpeed=80, .earlyExitRange = 2}, false); // NEEDS TO BE ADJUSTED BY SCRAPER LENGTH
//   //Loader
//   chassis.moveToPoint(31.869,-20.644, 1000, {.maxSpeed=60,.minSpeed=10}, false); //Wait Time Here Pretty Much Decides How Long It Spends In Loader\
//   //Long Goal
//   chassis.moveToPoint(31.869, 15.526+2, 1000, {.forwards = false, .minSpeed=40}, false);
//   //Score
//   hood.set_value(true);
//   scraper.set_value(false);
//   pros::delay(2500); //control delay time here
//   //Move Right A bit
//   chassis.moveToPoint(31.869, 13.026, 150, {.minSpeed=60}, false);
//   hood.set_value(false);
//   //Turn
//   chassis.turnToHeading(90, 600, {}, false);
//   //move Up
//   chassis.moveToPoint(41.587-2+7, 13.026, 400, {}, false);  // the x coordinate needs to be changed
//   //Descore
//   chassis.turnToHeading(180, 600, {}, false);
//   chassis.moveToPoint(41.587-2+7, 39.526,1000,{.forwards=false},false);
// }
// void right() { //Right We Run This
//   //200 100 200 500 450 600 500 1000 1000 500 50 1000 1000 1000 1000 2500
//   //200 100 200 500 450 600 500 1000 100 500 50 700 650 1500 1000 5000
//   codeVersion=6700;
//   wings.set_value(true);
//   scraper.set_value(false);
//   chassis.setPose(0, 0, 0);
//   //Moves ahead a little
//   chassis.moveToPoint(0,5,200,{}, false);
//   pros::delay(100);
//   intakeMotor.move(127);
//   midStageMotor.move(127);
//   finalStageMotor.move(127);
//   //Gets first blocks
//   chassis.turnToPoint(6.611, 24.172, 200, {}, false);
//   chassis.moveToPoint(6.611, 24.172, 500, {.maxSpeed=60}, true);
//   pros::delay(850);
//   scraper.set_value(true);
//   pros::delay(400);
//   //In between
//   chassis.turnToPoint(33.001, 10.24, 500, {}, false);
//   chassis.moveToPoint(33.001, 10.24, 1250, {.maxSpeed = 60}, false);
//   pros::delay(400);
//   //Align with long goal
//   chassis.turnToHeading(180, 500, {}, false);
//   pros::delay(50);
  
//   //Loader
//   wings.set_value(true);
  
//   scraper.set_value(true);
 
//   chassis.moveToPoint(33.001, -20.808,1250, {.maxSpeed = 50}, false);
//   pros::delay(300);
//   //Long Goal

  
  

//   chassis.moveToPoint(33.001, 21.759, 1500, {.forwards = false, .maxSpeed = 60}, true);
//   pros::delay(1000);
//   hood.set_value(true);
//   pros::delay(2000);
//   //Descore
//   scraper.set_value(false);
//   chassis.setPose(0,0,180); //i reset the position here cause lowkey i dont want to think
//   chassis.moveToPoint(0,-7.5, 500, {}, false);
//   hood.set_value(false);
//   chassis.turnToHeading(90,500, {}, false);
//   chassis.moveToPoint(12,-7.5, 500, {}, false);
//   chassis.turnToHeading(180,500, {}, false);
//   wings.set_value(false);
//   chassis.moveToPoint(12.8,18, 1500, {.forwards = false, .maxSpeed = 60}, false);
//   }
// void rightBlocks(){
// 	codeVersion=6700;
//   wings.set_value(true);
//   scraper.set_value(false);
//   chassis.setPose(0, 0, 0);
//   //Moves ahead a little
//   chassis.moveToPoint(0,5,200,{}, false);
//   pros::delay(100);
//   intakeMotor.move(127);
//   midStageMotor.move(127);
//   finalStageMotor.move(127);
//   //Gets first blocks
//   chassis.turnToPoint(6.611, 24.172, 200, {}, false);
//   chassis.moveToPoint(6.611, 24.172, 500, {.maxSpeed=60}, true);
//   pros::delay(850);
//   scraper.set_value(true);
//   pros::delay(400);
//   //In between
//   chassis.turnToPoint(33.001, 10.24, 500, {}, false);
//   chassis.moveToPoint(33.001, 10.24, 1250, {.maxSpeed = 60}, false);
//   pros::delay(400);
//   //Align with long goal
//   chassis.turnToHeading(180, 500, {}, false);
//   pros::delay(50);
  
//   //Loader
//   wings.set_value(true);
  
//   scraper.set_value(true);
 
//   chassis.moveToPoint(33.001, -20.808,1250, {.maxSpeed = 50}, false);
//   pros::delay(300);
//   //Long Goal

  

//   chassis.moveToPoint(33.001, 21.759, 1500, {.forwards = false, .maxSpeed = 60}, true);
//   pros::delay(1000);
//   hood.set_value(true);
//   pros::delay(2000);
//   //Blocks stuff
//   scraper.set_value(false);
//   chassis.setPose(0,0,180); //i dont want to think man
//   chassis.moveToPoint(0,-15, 500, {}, false);
//   hood.set_value(false);
//   chassis.moveToPoint(0,0,1500,{.forwards = false, .minSpeed=80},false);
// }
// void left(){ //3+4NOTCONSISTENT
//   /*chassis.moveTo(0, 0, 5000); X
// chassis.moveTo(0, 31.024, 5000); X
// chassis.moveTo(-13.575, 31.024, 5000); X 
// chassis.moveTo(20.334, 31.024, 5000); X
// chassis.moveTo(14.628, 31.024, 5000);X
// chassis.moveTo(14.628, 16.124, 5000);X
// chassis.moveTo(23.628, 8.124, 5000); X
// chassis.moveTo(38.628, -6.876, 5000); X
// chassis.moveTo(10.978, 20.774, 5000); X
// chassis.moveTo(39.628, 20.774, 5000);
// */
//   wings.set_value(true);
//   scraper.set_value(false);
//   hood.set_value(false);
//   chassis.setPose(0, 0, 0);
//   intakeMotor.move(127);
//   midStageMotor.move(127);
//   finalStageMotor.move(127);
//   //move inbetween
  
//   chassis.moveToPoint(0,32.924,1200,{},false);
//   //move to the loader
//   chassis.turnToPoint(-13.575, 32.924, 550, {}, false);
//   scraper.set_value(true);
//   pros::delay(100);
//   chassis.moveToPoint(-13.575, 32.924, 500 + 500,{},false);
//   //move To Long Goal
//   chassis.moveToPoint(20.334, 32.924, 1000, {.forwards = false}, false);
//   hood.set_value(true);
//   pros::delay(1000);
//   //move a bit
//   chassis.moveToPoint(10.628, 32.924, 600, {}, false);
//   scraper.set_value(false);
//   hood.set_value(false);
//   //move up
//   chassis.turnToPoint(10.628, 16.124, 550, {}, false);
//   chassis.moveToPoint(10.628, 16.124, 600, {}, false);
//   //get blocks
//   chassis.turnToPoint(23.628, 8.124, 300, {}, false);
//   chassis.moveToPoint(23.628, 8.124, 900, {}, true);
//   pros::delay(700);
//   scraper.set_value(true);
//   pros::delay(200);
//   //mid Goal
//   chassis.turnToPoint(37.628, -5.876, 750, {.forwards = false}, false);
//   chassis.moveToPoint(37.628, -5.876, 1200, {.forwards = false}, false);
//   //score
//   intakeMotor.move(-127);
//   midStageMotor.move(-127);
//   finalStageMotor.move(-127);
//   pros::delay(200);
//   intakeMotor.move(127);
//   midStageMotor.move(40);
//   finalStageMotor.move(-50);
//   pros::delay(1200);
//   //move back
//   chassis.turnToPoint(9.978, 25, 500, {}, false);
//   chassis.moveToPoint(9.978, 25, 1250, {}, false);
//   //descore
//   wings.set_value(false);
//   chassis.turnToPoint(30.628, 25, 300, {.forwards = false}, false);
//   chassis.moveToPoint(30.628, 25, 2000, {.forwards = false, .maxSpeed=70}, false);

// }
// void left2(){ //Lowk can delete this
//   codeVersion=6700;
//   wings.set_value(true);
//   scraper.set_value(false);
//   chassis.setPose(0, 0, 0);
//   intakeMotor.move(127);
//   midStageMotor.move(127);
//   finalStageMotor.move(127);
//   /*
// //   chassis.moveTo(0, 0, 5000);
// // chassis.moveTo(0, 31.19, 5000);
// // chassis.moveTo(-11.261, 31.19, 5000);
// // chassis.moveTo(20.498, 31.19, 5000);

// // */
//   chassis.moveToPoint(0,31.19,1000,{},false);
//   chassis.turnToHeading(270, 750, {}, false);
//   scraper.set_value(true);
//   pros::delay(250);
//   chassis.moveToPoint(-25.261,30.75, 1000, {.maxSpeed=45}, false);
//   chassis.setPose(0,0,270);
//   chassis.moveToPoint(44.498,0.1,1100, {.forwards=false,.maxSpeed=70}, false);
//   hood.set_value(true);
//   pros::delay(1500);
//   scraper.set_value(false);

//   float tempVarX = chassis.getPose().x;
//   chassis.moveToPoint(tempVarX-17.6,0,600,{},false);
//   hood.set_value(false);
//   chassis.setPose(0,0,270);
//   chassis.turnToHeading(135,700,{},false);
//   chassis.moveToPoint(24,-24,850,{}, true);
//   pros::delay(650);
//   scraper.set_value(true);
//   pros::delay(200);
//   pros::delay(500);
//   chassis.turnToHeading(315,1000, {}, false);
//   chassis.moveToPoint(44,-46,1500,{.forwards = false, .maxSpeed=127},true);
//   pros::delay(1500);
//   chassis.moveToPoint(44+5,-46-5,1000,{.forwards=false,.maxSpeed=40},true);
//   intakeMotor.move(-127);
//   midStageMotor.move(-127);
//   finalStageMotor.move(-127);
//   pros::delay(200);
//   intakeMotor.move(127);
//   midStageMotor.move(35);
//   finalStageMotor.move(-30);
//   pros::delay(1500);
//   chassis.moveToPoint(24,-24,850,{},true);
// }
// void leftBackup(){ //DESPITE BEINEG CALLED BACKUP, THIS IS OUR LEFT SEVEN!!!
//   codeVersion=6700;
//   wings.set_value(true);
//   scraper.set_value(false);
//   chassis.setPose(0, 0, 0);
//   //Moves ahead a little
//   chassis.moveToPoint(0,5,200,{}, false);
//   pros::delay(100);
//   intakeMotor.move(127);
//   midStageMotor.move(127);
//   finalStageMotor.move(127);
//   //Gets first blocks
//   chassis.turnToPoint(-6.611, 24.172, 200, {}, false);
//   chassis.moveToPoint(-6.611, 24.172, 500, {.maxSpeed=60}, true);
//   pros::delay(850);
//   scraper.set_value(true);
//   pros::delay(400);
//   //In between
//   chassis.turnToPoint(-33.001, 10.24, 500, {}, false);
//   chassis.moveToPoint(-33.001, 10.24, 1250, {.maxSpeed = 60}, false);
//   pros::delay(400);
//   //Align with long goal
//   chassis.turnToHeading(180, 500, {}, false);
//   pros::delay(50);
  
//   //Loader
//   wings.set_value(true);
  
//   scraper.set_value(true);
 
//   chassis.moveToPoint(-33.001, -20.808,1250, {.maxSpeed = 50}, false);
//   pros::delay(300);
//   //Long Goal

  
  

//   chassis.moveToPoint(-33.001, 21.759, 1500, {.forwards = false, .maxSpeed = 60}, true);
//   pros::delay(1000);
//   hood.set_value(true);
//   pros::delay(2000);
//   //Descore
//   scraper.set_value(false);
//   chassis.setPose(0,0,180); //i reset the position here cause lowkey i dont want to think
//   chassis.moveToPoint(0,-7.5, 500, {}, false);
//   hood.set_value(false); 
//   chassis.turnToHeading(90,500, {}, false);
//   chassis.moveToPoint(12,-7.5, 500, {}, false);
//   chassis.turnToHeading(180,500, {}, false);
//   wings.set_value(false);
//   chassis.moveToPoint(12.8,18, 1500, {.forwards = false, .maxSpeed = 60}, false);
// }
// void SAWP()
// {
//   //put sawp here
// }
// void skill2(){ //

// }
// void skill(){ //This is our 20 point Skills, make sure to rename it to just skill() before running.
//   intakeMotor.move(-127);
//   midStageMotor.move(-127);
//   finalStageMotor.move(-127);
//   chassis.setPose(0,0,0);
//   chassis.moveToPoint(0,-18,1000,{.forwards=false},false);
//   chassis.moveToPoint(0,36,8000,{.minSpeed=127,.earlyExitRange=1},true);
//   pros::delay(200);
//   scraper.set_value(true);
//   pros::delay(4500);
//   scraper.set_value(false);
// }
// void skill3(){

//   codeVersion=6700;
//   wings.set_value(true);
//   scraper.set_value(false);
//   chassis.setPose(0, 0, 0);
//   //Moves ahead a little
//   chassis.moveToPoint(0,5,200,{}, false);
//   pros::delay(100);
//   intakeMotor.move(127);
//   midStageMotor.move(127);
//   finalStageMotor.move(127);
//   //Gets first blocks (inconsistent part)
//   chassis.turnToPoint(-6.611, 24.172, 200, {}, false);
//   chassis.moveToPoint(-6.611, 24.172, 500, {.maxSpeed=80}, true);
//   pros::delay(650);
//   scraper.set_value(true);
//   pros::delay(400);
//   // In between
//   chassis.turnToPoint(-33.001, .24, 600, {}, false);
//   chassis.moveToPoint(-33.001, .24, 1350, {.maxSpeed = 60}, false);
//   pros::delay(400);
//   //Align with long goal
//   chassis.turnToHeading(180, 500, {}, false);
//   pros::delay(50);
  
//   //Loader
//   wings.set_value(true);
  
//   scraper.set_value(true);
 
//   chassis.moveToPoint(-33.001, -20.808,1250, {.maxSpeed = 50}, false);
//   pros::delay(2050);
//   //Long Goal

  
  

//   chassis.moveToPoint(-33.001, 21.759, 1500, {.forwards = false, .maxSpeed = 60}, true);
//   pros::delay(1000);
//   hood.set_value(true);
//   pros::delay(4000);
//   //Descore
//   scraper.set_value(false);
//   chassis.setPose(0,0,180); //i reset the position here cause lowkey i dont want to think
//   chassis.moveToPoint(0,-24, 1000, {}, false);
//   hood.set_value(false);
//   chassis.turnToPoint(14,-24,600, {}, false);
//   chassis.moveToPoint(14,-24,1500, {}, false);
//   chassis.turnToHeading(180,800,{},false);
//   chassis.moveToPoint(14,-30,1000,{},false);
//   float tempVarY = chassis.getPose().y;
//   chassis.turnToPoint(14+50,tempVarY-6,800,{},false);
  
//   chassis.moveToPoint(14+50 ,tempVarY-6, 8000, {.minSpeed=127,.earlyExitRange=1},true);
//   intakeMotor.move(-127);
//   midStageMotor.move(-127);
//   finalStageMotor.move(-127);

  
// }
 void autonomous() {
//   autonomousRunning = true;
//     switch(autonomousValue){
//     case 0:
//       skill();
//       break;
//     case 1:
//       right();
//       break;
//     case 2:
//       left();
//       break;
//     case 3:
// 	  rightBlocks();
// 	  break;
//     case 4:
//     leftBackup();
//     break;
//     case 5:
//     SAWP();
//     break;
//     }
 }

/**
 * Runs in driver control
 */
 void driverControl()
 {

  float delayTime = 10;
  //Variables for presses
  bool R1 = false;
  bool X = false;
  bool R2 = false;
  bool L1 = false;
  while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);

    //toggling variables
	  if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
		R1 = !R1;
	  }
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
		L1 = !L1;
	  }
    
    //INTAKE BUTTONS

    //A Button Hold
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
      R1 = false;
      redirect.set_value(false);
      firstStageMotor.move(127);
      secondStageMotor.move(-42);
    }

    //L2 Hold
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      R1 = false;
      intakeLift.set_value(true);
      firstStageMotor.move(127*-5/12);
      secondStageMotor.move(127*5/12);
    }

    //R1 Toggle
    else if (R1){
        intakeLift.set_value(true);
        firstStageMotor.move(127);
		    secondStageMotor.move(-127);
      } 
    else if (!R1){
      intakeLift.set_value(false);
      firstStageMotor.move(0);
      secondStageMotor.move(0);
    }

    
    //PNEUMATICS
    //L1 Toggle
    if (L1){
        scraper.set_value(true);
    }
    else if (!L1){
        scraper.set_value(false);
    }
    //X Hold
    //Basically, you press X the first time, it sets redirect to false, then the program goes on and delays 10 milliseconds, then sets it to true

    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) && !X){
      pros::Task screenTask([&]() {
        X = true;
        redirect.set_value(false);
        pros::delay(100);
        redirect.set_value(true);
        X = false;
      });
    }
    
    //R2 Hold
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      R2 = true;
      wings.set_value(true);
    }
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && R2 == true){
      wings.set_value(false);
    } 
    else if (R2 == false){
      wings.set_value(false);
    }
    else{
      wings.set_value(true);
    }

    pros::delay(delayTime);

  
}
 }


void PIDTASK(){
  pros::Task PIDTASK ([&]() {
    //pros::lcd::print(5, "P: %f", linearController.kP); // x
    //pros::lcd::print(6, "I: %f", linearController.kI); // y
    //pros::lcd::print(7, "D: %f", linearController.kD); // heading

    pros::lcd::print(5, "P: %f", angularController.kP); // x
    pros::lcd::print(6, "I: %f", angularController.kI); // y
    pros::lcd::print(7, "D: %f", angularController.kD); // heading
	pros::delay(20);
  });
}

void PID(){
    chassis.setPose(0,0,0);
    //chassis.moveToPoint(0,24,2000);
    chassis.turnToHeading(90,300);
}
bool testingAutonomous = false;
bool PIDBOOL = false;
void opcontrol() {
    
    // controller
    // loop to continuously update motors
    //autonomous();

    //driverControl();
   
    //This only runs if driver control and stuff isn't running
	
    if(PIDBOOL){
      PIDTASK();
      PID();
    }
    else if(testingAutonomous){
      competition_initialize();
      while(!autonomousRunning){
      if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
        autonomousRunning = true;
        pros::delay(500);
        autonomous();
      }
      pros::delay(100);
    }
    }
    else{
      driverControl();
    }
  
}
