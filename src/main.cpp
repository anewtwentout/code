#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
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
pros::adi::DigitalOut redirect ('C', false);
// motor groups
pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({14, 15, 16}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)
pros::Motor firstStageMotor(1, pros::MotorGearset::blue);
pros::Motor secondStageMotor(2, pros::MotorGearset::blue);

// Inertial Sensor on port 17
pros::Imu imu(17);
//Odom
pros::Rotation vertical_encoder(-18);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -0.4);
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

lemlib::ControllerSettings linearController( 7, // proportional gain (kP)
                                            0, // integral gain (kI)
                                           40, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller

lemlib::ControllerSettings angularController(1.9, // proportional gain (kP)
                                             0,// integral gain (kI)
                                            12, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel
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
int autonomousValue =6; // 0 = SAWP, 1 = right control rush, 2 = left control rush, 3 = right 3+4, 4 = left 3 +4, 5 = Right 7 Push
int amountOfAutonomousCodes = 6;

void changeAutoValue(){
pros::screen_touch_status_s_t screenTouch = pros::screen::touch_status();
  pros::lcd::print(5, "XValue: %d", screenTouch.x); //making sure its working all right
  if(screenTouch.x>=200){
    autonomousValue = (autonomousValue+1)%amountOfAutonomousCodes;
  }
  if(screenTouch.x<200){
    autonomousValue = (autonomousValue-1)%amountOfAutonomousCodes;
  }
  
}
void competition_initialize() {
  pros::screen::touch_callback(changeAutoValue, TOUCH_PRESSED);
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
      pros::lcd::print(0, "Autonomous: %s", "SAWP");
      break;
    case 1:
      pros::lcd::print(0, "Autonomous: %s", "Right 4L");
      break;
    case 2:
      pros::lcd::print(0, "Autonomous: %s", "Left 4L");
      break;
    case 3:
      pros::lcd::print(0, "Autonomous: %s", "Right 4L+3M");
      break;
    case 4:
      pros::lcd::print(0, "Autonomous: %s", "Left 4L+3M");
      break;
    case 5:
      pros::lcd::print(0, "Autonomous: %s", "Right 7L Push");
      break;
       
  }
  pros::delay(10);
  }
});

}

void SAWP(){
  //setup
  chassis.setPose(0,0,0); //X and Y might be changed btw,
  wings.set_value(true);
  redirect.set_value(false);
  firstStageMotor.move(127);
  secondStageMotor.move(-127);
  //Right Inbetween
  chassis.moveToPoint(0,34.5,950,{},false);
  //Right Match Loader
  scraper.set_value(true);
  chassis.turnToPoint(17.969,34.75,570,{},false);
  chassis.moveToPoint(19.969,34.75,685,{.maxSpeed=40},false);
  //Right Long Goal
  chassis.turnToPoint(-20.871,36,250,{.forwards = false}, false);
  chassis.moveToPoint(-20.871,36, 1000, {.forwards = false, .minSpeed = 60}, true);
  pros::delay(700);
  wings.set_value(false);
  scraper.set_value(false);
  pros::delay(850);
  //First Cluster
  chassis.turnToPoint(-23.644,9.01,1000, {.minSpeed=40}, false);
  wings.set_value(true);
  chassis.moveToPoint(-23.644, 9.01, 500, {.maxSpeed=40}, true);
  pros::delay(300);
  //scraper.set_value(true);
  //pros::delay(700);}
//   //Second Cluster
  chassis.turnToPoint(-21.644, -38.99, 150, {}, false);
  chassis.moveToPoint(-21.644, -30.99, 600,  {.minSpeed=50}, true);
  chassis.moveToPoint(-21.644, -42.99, 150, {.maxSpeed=40}, false);
  pros::delay(280);
 // scraper.set_value(true);
 // scraper.set_value(true);
  pros::delay(0520);
  //Left Inbetween
  chassis.turnToPoint(0,-63.5, 350, {}, false);
  chassis.moveToPoint(0,-63.5, 950, {}, false);
  //Left Long Goal
  chassis.turnToPoint(-20.905, -63.5, 450, {.forwards = false}, false);
  chassis.moveToPoint(-20.905, -63.5, 200, {.forwards = false}, true);
  chassis.moveToPoint(-20.905, -63.5, 300, {.forwards = false,.maxSpeed=60}, true);
  pros::delay(300);
  scraper.set_value(true);
  wings.set_value(false);
  pros::delay(1200);
  //Left MatchLoader
  chassis.turnToPoint(18.032, -62.25, 250, {}, false);
  chassis.moveToPoint(18.032, -62.25, 400, {.minSpeed = 127, .earlyExitRange = 3}, false);
  wings.set_value(true);
  chassis.moveToPoint(18.032,-62.25, 950, {.maxSpeed = 45}, false);
  //Before Mid Goal
  // chassis.turnToPoint(-23.644,-38.99, 450, {.forwards=false}, false);
  // chassis.moveToPoint(-23.644,-38.99,1000, {.forwards = false}, false);
  // scraper.set_value(false);
  // pros::delay(10000);
  //Mid Goal
  //chassis.turnToPoint(-36.644, -23.99, 500, {.forwards = false}, false);
 
  chassis.moveToPoint(6, -62.99,  300,{.forwards=false}, false);
  chassis.turnToPoint(36.644+3+0.5, -23.99-3, 100, {.forwards = false}, false);
  chassis.moveToPose(-36.644+3+0.5, -23.99-3, 135, 1680, {.forwards = false, .minSpeed = 50}, true);
  pros::delay(1680);
  redirect.set_value(true);
  firstStageMotor.move(127);
  secondStageMotor.move(-45);}
void rightControlRush()
{
// chassis.moveTo(0, 0, 5000);
// chassis.moveTo(0, 32.01, 5000);
// chassis.moveTo(17.969, 32.01, 5000);
// chassis.moveTo(-20.871, 33.01, 5000);
// chassis.moveTo(-20.936, 42.679, 5000);
// chassis.moveTo(-36.015, 42.679, 5000);

//setup
  chassis.setPose(0,0,0); //X and Y might be changed btw,
  wings.set_value(true);
  redirect.set_value(false);
  firstStageMotor.move(127);
  secondStageMotor.move(-127);
  //Right Inbetween
  chassis.moveToPoint(0,35.51,950,{},false);
  //Right Match Loader
  scraper.set_value(true);
  chassis.turnToPoint(17.969,35.51,570,{},false);
  chassis.moveToPoint(19.969,35.51,695,{.maxSpeed=40},false);
  //Right Long Goal
  chassis.turnToPoint(-20.871,36.01,250,{.forwards = false}, false);
  chassis.moveToPoint(-20.871,36.01, 1000, {.forwards = false, .minSpeed = 60}, true);
  pros::delay(650);
  wings.set_value(false);
  scraper.set_value(false);
  pros::delay(800);
  //wing
  chassis.moveToPoint(-8, 36.01, 700,{.minSpeed=70}, false);
  chassis.turnToPoint(-16.936, 45.679,350,{.forwards=false}, false);
  chassis.moveToPoint(-16.936, 45.679,620,{.forwards=false,.minSpeed=70}, false);
  chassis.turnToPoint(-36.015, 45.679, 350,{.forwards=false}, false);
  chassis.moveToPoint(-41.015, 45.679, 1200,{.forwards=false,.maxSpeed=110, .minSpeed=70}, false);
  leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  leftMotors.brake();
  rightMotors.brake();
}
void leftControlRush(){
  chassis.setPose(0,0,0); //X and Y might be changed btw,
  wings.set_value(true);
  redirect.set_value(false);
  firstStageMotor.move(127);
  secondStageMotor.move(-127);
   //InBetween
  chassis.moveToPoint(0,35.51, 950, {}, false);
  //MatchLoader
  scraper.set_value(true);
  chassis.turnToPoint(-17.875,35.51, 400, {}, false);
  chassis.moveToPoint(-17.875, 35.51, 850, {.maxSpeed=50}, false);
  //Long Goal
  // chassis.turnToPoint(23.006,35.51, 400, {.forwards=false}, false);
  chassis.moveToPoint(23.006, 35.51, 1200, {.forwards = false}, true);
  pros::delay(700);
  wings.set_value(false);
  scraper.set_value(false);
  pros::delay(850);
  //wing
  chassis.moveToPoint(8, 36.01, 700,{.minSpeed=70}, false);
  chassis.turnToPoint(16.936, 26.341,350,{.forwards=false}, false);
  chassis.moveToPoint(16.936, 26.341,620,{.forwards=false,.minSpeed=70}, false);
  chassis.turnToPoint(36.015, 26.341, 350,{.forwards=false}, false);
  chassis.moveToPoint(41.015, 26.341, 1200,{.forwards=false,.maxSpeed=110, .minSpeed=70}, false);
  leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  leftMotors.brake();
  rightMotors.brake();
}
void rightFourPlusThree()
{
// chassis.moveTo(0, 0, 5000);
// chassis.moveTo(0, 32.01, 5000);
// chassis.moveTo(17.969, 32.01, 5000);
// chassis.moveTo(-20.871, 33.01, 5000);
// chassis.moveTo(-23.644, 9.01, 5000);
// chassis.moveTo(-36.612, -4.33, 5000);

//setup
  chassis.setPose(0,0,0); //X and Y might be changed btw,
  wings.set_value(true);
  redirect.set_value(false);
  firstStageMotor.move(127);
  secondStageMotor.move(-127);
  //Right Inbetween
  chassis.moveToPoint(0,35.51,950,{},false);
  //Right Match Loader
  scraper.set_value(true);
  chassis.turnToPoint(17.969,35.51,400,{},false);
  chassis.moveToPoint(19.969,35.51,695,{.maxSpeed=40},false);
  //Right Long Goal
  chassis.turnToPoint(-20.871,36.01,250,{.forwards = false}, false);
  chassis.moveToPoint(-20.871,36.01, 1000, {.forwards = false, .minSpeed = 60}, true);
  pros::delay(650);
  wings.set_value(false);
  scraper.set_value(false);
  pros::delay(800);
  //Cluster
  chassis.turnToPoint(-26.533,6.112,1000, {.minSpeed=40}, false);
  wings.set_value(true);
  secondStageMotor.move(0);
  chassis.moveToPoint(-26.533, 6.112, 500, {.maxSpeed=40}, false);
  //scraper.set_value(true);
  //pros::delay(300);
  //MidGoal
  chassis.turnToPoint(-37+2, -2.5+4, 500, {}, false);
  scraper.set_value(false);
  chassis.moveToPoint(-37+2, -2.5+4, 1200, {}, false);
  intakeLift.set_value(true);
  firstStageMotor.move(-80);
  secondStageMotor.move(80);
  scraper.set_value(false);
  pros::delay(1600);
  //Wing
  chassis.moveToPose(-10.629, 47.5,90,1450, {.forwards = false, .minSpeed=100}, false);
  chassis.turnToPoint(-44.967,47.5,500,{.forwards=false},false);
  wings.set_value(false);
  chassis.moveToPoint(-28.967, 47.5, 1000, {.forwards=false, .minSpeed = 100, .earlyExitRange=5}, false);
  chassis.moveToPoint(-44.967, 47.5, 1000, {.forwards=false, .maxSpeed = 40}, false);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}
void leftThreePlusFour(){
  /*chassis.moveTo(0, 0, 5000);
chassis.moveTo(0, 31.665, 5000);
chassis.moveTo(-17.875, 31.665, 5000);
chassis.moveTo(23.006, 32.165, 5000);
chassis.moveTo(24.231, 7.787, 5000);
chassis.moveTo(34.853, -2.835, 5000);
chassis.moveTo(10.859, 23.159, 5000);
chassis.moveTo(33.853, 23.159, 5000);



*/
  //setup
  chassis.setPose(0,0,0); //X and Y might be changed btw,
  wings.set_value(true);
  redirect.set_value(false);
  firstStageMotor.move(127);
  secondStageMotor.move(-127);
  //InBetween
  chassis.moveToPoint(0,35.51, 950, {}, false);
  //MatchLoader
  scraper.set_value(true);
  chassis.turnToPoint(-17.875,35.51, 440, {}, false);
  chassis.moveToPoint(-17.875, 35.51, 850, {.maxSpeed=50}, false);
  //Long Goal
  // chassis.turnToPoint(23.006,35.51, 400, {.forwards=false}, false);
  chassis.moveToPoint(23.006, 36.01, 1200, {.forwards = false}, true);
  pros::delay(700);
  wings.set_value(false);
  scraper.set_value(false);
  pros::delay(850);
  //Cluster
  chassis.turnToPoint(22.231,7.787,800, {.minSpeed=40}, false);
  wings.set_value(true);
  chassis.moveToPoint(22.231,7.787,900,{.minSpeed=70,.earlyExitRange=5}, true);
  
  scraper.set_value(true);
  chassis.moveToPoint(22.231,7.787, 150,{.maxSpeed=70}, false);
 
  scraper.set_value(true);
   pros::delay(300);
  //MidGoal
  chassis.turnToPoint(34.853, -2.834, 600, {.forwards=false}, false);
  chassis.moveToPose(34.853,-2.83,315,750,{.forwards = false, .minSpeed=60},true);
  pros::delay(550);
  redirect.set_value(true);
  pros::delay(200);
  firstStageMotor.move(127);
  secondStageMotor.move(-60);
  pros::delay(650);
  scraper.set_value(false);
  //Wing
  chassis.turnToPoint(10.859,27.05, 350, {}, false);
  chassis.moveToPoint(10.859, 27.05, 1000, {.minSpeed = 49, .earlyExitRange = 0.5}, false);
  chassis.turnToPoint(35.859,28.05,500,{.forwards=false},false);
  wings.set_value(false);
  chassis.moveToPoint(35.859,28.05, 1500, {.forwards=false,.minSpeed=127, .earlyExitRange=20});
  chassis.moveToPoint(35.859,28.05, 1500, {.forwards=false,.maxSpeed=60});

  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}
void rightSeven()
{
// chassis.moveTo(0, 0, 5000);
// chassis.moveTo(-27.052, 10.922, 5000);
// chassis.moveTo(-7.393, 30.581, 5000);
// chassis.moveTo(13.274, 34.614, 5000);
// chassis.moveTo(-20.835, 35.286, 5000);
// chassis.moveTo(-11.258, 35.286, 5000);
// chassis.moveTo(-19.659, 43.351, 5000);
// chassis.moveTo(-40.158, 42.679, 5000);



//setup
chassis.setPose(0,0,0); //X and Y might be changed btw,
wings.set_value(true);
redirect.set_value(false);
firstStageMotor.move(127);
secondStageMotor.move(-127);
//first cluster
chassis.turnToPoint(6,20,300,{},false);
chassis.moveToPoint(6,20,900,{},false);
scraper.set_value(true);
chassis.turnToPoint(22,1,300,{},false);
chassis.moveToPoint(22,1,700,{},false);

}
void autonSkills()
{
  //setup
chassis.setPose(0,0,0);
firstStageMotor.move(127);
secondStageMotor.move(-127);
wings.set_value(true);
redirect.set_value(false);
//first cluster
chassis.moveToPoint(0, 25.5, 1000,{.maxSpeed=80},false);
firstStageMotor.move(127);
secondStageMotor.move(-127);
chassis.moveToPoint(0, 37, 600,{.maxSpeed=50},true);
pros::delay(250);
scraper.set_value(true);
pros::delay(350);
chassis.turnToHeading(-90, 650);
chassis.moveToPoint(20, 37, 800,{.forwards=false},false);
redirect.set_value(true);
//firstStageMotor.move(-30);
secondStageMotor.move(-60);
pros::delay(500);
redirect.set_value(false);
//move to matchload
scraper.set_value(false);
secondStageMotor.move(0);
chassis.moveToPoint(-36,37,1300,{},false);
//pose reset shit
lemlib::Pose pose = chassis.getPose();
chassis.setPose(pose.x,pose.y,pose.theta-45);
chassis.turnToHeading(180, 500,{},true);
pros::delay(100);
scraper.set_value(true);
pros::delay(400);
//matchload
chassis.moveToPoint(-36, 20, 800,{.forwards=true,.maxSpeed=60},false);
//wings.set_value(false);
}

 void autonomous() {
   autonomousRunning = true;
     switch(autonomousValue){
     case 0:
       SAWP();
       break;
    case 1:
      rightControlRush();
      break;
    case 2:
      leftControlRush();
      break;
    case 3:
      rightFourPlusThree();
      break;
    case 4 :
      leftThreePlusFour();
      break;
    case 5:
      rightSeven();
      break;
    case 6:
      autonSkills();
      break;
     }
     
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
    //INTAKE BUTTONS

    //A Button Hold
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
      R1 = false;
      redirect.set_value(true);
      /*
      //match:
      firstStageMotor.move(127);
      secondStageMotor.move(-82);
      */
      //skills:
      firstStageMotor.move(127);
      secondStageMotor.move(-42);
      
    }

    //L2 Hold
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      R1 = false;
      L2 = true;
      intakeLift.set_value(true);
      /*
      //match
      firstStageMotor.move(-80);
      secondStageMotor.move(127);
      */
      //skills:
      firstStageMotor.move(-40);
      secondStageMotor.move(127);
      

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
