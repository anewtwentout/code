#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.h"
#include <numbers>
#include "auto.hpp"
#include "globals.hpp"
//variables
// controller

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
void disabled() {


}

/**
 * runs after initialize if the robot is connected to field control
 */

bool autonomousRunning = false;
int amountOfAutonomousCodes = 8;

// void changeAutoValue(){
// pros::screen_touch_status_s_t screenTouch = pros::screen::touch_status();
//   pros::lcd::print(5, "XValue: %d", screenTouch.x); //making sure its working all right
//   if(screenTouch.x>=200){
//     autonomousValue = (autonomousValue+1)%amountOfAutonomousCodes;
//   }
//   if(screenTouch.x<200){
//     autonomousValue = (autonomousValue-1)%amountOfAutonomousCodes;
//   }
  
// }
void competition_initialize() {
//   // pros::screen::touch_callback(changeAutoValue, TOUCH_PRESSED);
//   // pros::Task autonomousTask([&]() {
//   // while(true)
//   // {
//   // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
//   //   autonomousValue = (autonomousValue+1)%amountOfAutonomousCodes;
//   // }
//   // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
//   //   autonomousValue = (autonomousValue-1)%amountOfAutonomousCodes;
//   // }
  
//   switch(autonomousValue){
//     case 0:
//       pros::lcd::print(0, "Autonomous: %s", "SAWP");
//       break;
//     case 1:
//       pros::lcd::print(0, "Autonomous: %s", "Right 4L");
//       break;
//     case 2:
//       pros::lcd::print(0, "Autonomous: %s", "Left 4L");
//       break;
//     case 3:
//       pros::lcd::print(0, "Autonomous: %s", "Right 4L+3M");
//       break;
//     case 4:
//       pros::lcd::print(0, "Autonomous: %s", "Left 4L+3M");
//       break;
//     case 5:
//       pros::lcd::print(0, "Autonomous: %s", "Right 7L Push");
//       break;

       
//   }
//   pros::delay(10);
//   }
// });

}










 

 
 
/**
 * Runs in driver control
 */
 void driverControl()
 {
  redirect.set_value(false);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  //chassis.setPose(0,0,0);
  float delayTime = 10;
  //Variables for presses
  bool R1 = false;
  bool X = false;
  bool R2 = false;
  bool L1 = false;
  bool L2 = false;
  bool UP = false;
  while (true) {
    
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
	  if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
		UP = !UP;
	}
    //INTAKE BUTTONS

    //A Button Hold
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
      R1 = false;
      redirect.set_value(true);
      
      //match:
      firstStageMotor.move(127);
      secondStageMotor.move(-82-14);
     
      //skills:
    //   firstStageMotor.move(127);
    //   secondStageMotor.move(-42);
      
    }

    //L2 Hold
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      R1 = false;
      L2 = true;
      intakeLift.set_value(true);
      //
      //match
      firstStageMotor.move(-80);
      secondStageMotor.move(127);
      //
      //skills:
    //   firstStageMotor.move(-40);
    //   secondStageMotor.move(127);
      

    }

    //R1 Toggle
    else if (R1){
        firstStageMotor.move(127);
		    secondStageMotor.move(-127);
        redirect.set_value(false);
      } 
    else if (!R1){
      firstStageMotor.move(0);
      secondStageMotor.move(0);
    }
    //Things to do when you release a button
    if (L2 && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      intakeLift.set_value(false);
      L2 = false;
    }
    
    //PNEUMATICS
    //L1 Toggle
    if (L1){
        scraper.set_value(true);
    }
    else if (!L1){
        scraper.set_value(false);
    }
	//UP Toggle
	if(UP){
		midGoal_descore.set_value(true);
	}
	else if(!UP){
		midGoal_descore.set_value(false);
	}
    //X Hold
    //Basically, you press X the first time, it sets redirect to false, then the program goes on and delays 10 milliseconds, then sets it to true

    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) && !X){
      pros::Task screenTask([&]() {
        X = true;
        redirect.set_value(true);
        pros::delay(100);
        redirect.set_value(false);
        X = false;
      });
    }
    //UP ARROW
	
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
    //chassis.moveToPoint(0,24,1000);
    chassis.turnToHeading(90,1000);
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