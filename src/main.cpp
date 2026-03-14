#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/motors.h"
#include <numbers>
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
//Distance Sensors
pros::Distance frontDistance(8);
pros::Distance rightDistance(9);
pros::Distance leftDistance(10);
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
void disabled() {


}

/**
 * runs after initialize if the robot is connected to field control
 */

bool autonomousRunning = false;
int autonomousValue =7; // 0 = SAWP, 1 = right control rush, 2 = left control rush, 3 = right 3+4, 4 = left 3 +4, 5 = Right 7 Push, 6 = autonSkills 7 = skills96
int amountOfAutonomousCodes = 8;

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
chassis.setPose(0,0,270); //X and Y might be changed btw,
wings.set_value(true);
redirect.set_value(false);
firstStageMotor.move(127);
secondStageMotor.move(-127);

//First Cluster
chassis.moveToPoint(-23.644,9.01,600,{.minSpeed=80,.earlyExitRange=2},false);
scraper.set_value(true);
//Inbetween
chassis.turnToPoint(0,34.75,500,{.forwards=false,.minSpeed=45,.earlyExitRange=5},false);
chassis.moveToPoint(0,34.75,1000,{.forwards=false},false);
//Matchloader
chassis.turnToPoint(17.969,34.75,850,{},false);
chassis.moveToPoint(17.969,34.75,1000,{},false);
//Long Goal
chassis.turnToPoint(-20.871,36,250,{.forwards = false}, false);
chassis.moveToPoint(-15,36,700,{.forwards=false,.minSpeed=127,.earlyExitRange=3});
chassis.moveToPoint(-20.871,36, 500+850, {.forwards = false}, true);
pros::delay(500);
wings.set_value(false);
scraper.set_value(false);
pros::delay(850);
//MoveBack
chassis.moveToPoint(-10,36,400,{.minSpeed=127,.earlyExitRange=1},false);
//wing with attempt at motion chaining
wings.set_value(false);
chassis.turnToPoint(-14,45,1000,{.forwards=false,.minSpeed=80,.earlyExitRange=10},false);
chassis.moveToPoint(-14,45,1000,{.forwards=false,.minSpeed=25,.earlyExitRange=1},false);
chassis.turnToPoint(-45,45,1000,{.forwards=false},false);
chassis.moveToPoint(-45,45,1000,{.forwards=false},false);
}

void autonSkills()
{
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  /*chassis.moveTo(0, 0, 5000);
chassis.moveTo(18.031, 18.031, 5000);
chassis.moveTo(26.163, 26.163, 5000);
chassis.moveTo(40.305, 12.021, 5000);
chassis.moveTo(0.707, 51.619, 5000);
*/
  //setup
chassis.setPose(0,0,45);
firstStageMotor.move(127);
secondStageMotor.move(-127);
wings.set_value(true);
redirect.set_value(false);
//First Left Cluster
chassis.moveToPoint(18.031, 18.031, 900,{},false);
firstStageMotor.move(127);
secondStageMotor.move(-127);
chassis.moveToPoint(26.163, 26.163, 700,{.maxSpeed=30},true);
//MidGoal 
chassis.turnToPoint(39.305+1.5-0.5,11.021-0.5,550, {.forwards=false},false);
chassis.moveToPose(39.305+1.5-0.5, 11.021-0.5,315, 1200,{.forwards=false},false);
redirect.set_value(true);
secondStageMotor.move(-80);
pros::delay(300);
firstStageMotor.move(0);
secondStageMotor.move(50);
//First Left Inbetween
scraper.set_value(false);
// secondStageMotor.move(-127);
chassis.moveToPoint(0,54.5+0.25-1+.25,1200,{},false);
redirect.set_value(false);
firstStageMotor.move(127);
secondStageMotor.move(-127);
//First Left Matchloader
scraper.set_value(true);
chassis.turnToPoint(-14.5, 54.5+0.25-1+.25,650,{},false);
chassis.moveToPoint(-14.5,54.5+0.25-1+.25,1600+500,{.maxSpeed=85},false);


//Move Outside
secondStageMotor.move(0);
chassis.moveToPoint(-0, 54.5,600,{.forwards=false},false);
chassis.turnToPoint(14,68-1.5,400,{.forwards=false},false);
chassis.moveToPoint(14,68-1.5,600,{.forwards=false},false);
//Move to Other Side
// scraper.set_value(false);
chassis.turnToPoint(96,68-1.5,500,{.forwards=false},false);
chassis.moveToPoint(96,68-1.5,1650,{.forwards=false},false);
//Move In Between
chassis.turnToPoint(96,52.5+0.75+0.5-0.75+0.1-0.5-.25-.4,600,{},false);//Long Goal
chassis.moveToPoint(96,52.5+0.75+0.5-0.75+0.1-0.5-.25-.4,700+400,{},false);//Long Goal
//Long Goal
chassis.turnToPoint(76-2+3,52.5+0.75+0.5-0.75+0.1-0.5-.25-.4,650,{.forwards=false},false);//Long Goal
chassis.moveToPoint(76-2+3,52.5+0.75+0.5-0.75+0.1-0.5-.25-.4,800,{.forwards=false},false);//Long Goal
//score
secondStageMotor.move(-127);
wings.set_value(false);
scraper.set_value(true);
pros::delay(1800);
//Matchloader
chassis.turnToPoint(111, 52.5+0.75+0.5-0.25-1.5-.5-1+.1,800, {}, false);
chassis.moveToPoint(101, 52.5+0.75+0.5-0.25-1.5-.5-1+.1,1000,{}, true);
pros::delay(600);
wings.set_value(true);
pros::delay(400);
chassis.moveToPoint(111,52.5+0.75+0.5-0.25-1.5-.5-1+.1,1400,{.maxSpeed=80},false);
//Long Goal
chassis.turnToPoint(76-2+3,52.5+0.75+0.5-0.75+0.1-0.5-.25-.4,500,{.forwards=false},false);
chassis.moveToPoint(76-2+3,52.5+0.75+0.5-0.75+0.1-0.5-.25-.4,800,{.forwards=false},false);//Long Goal
//Score
secondStageMotor.move(-127);
wings.set_value(false);
scraper.set_value(true);
pros::delay(2000);
//HitBalls back
// chassis.moveToPoint(76-2+12,52.5+0.75+0.5-0.75+0.1,800,{}, false);
// wings.set_value(true);
// chassis.moveToPoint(76-2+3,52.5+0.75+0.5-0.75+0.1-0.5-.25-.25-.125,1200,{.forwards=false,.maxSpeed=20},false);//Long Goal
//Guessing After This
wings.set_value(true);


//Move out A little
chassis.moveToPoint(90,52.5+0.75+0.5-0.75+0.1-0.5-.25-.125-.5,1000, {}, false);
//Inbetween Other Left side
chassis.turnToPoint(90,-43.25-2.5-0-6.5+2+0.125+.5-.25-.2,1500,{},false);
chassis.moveToPoint(90,-43.25-2.5-0-6.5+2+0.125+.5-.25-.2,2500,{},false);
//Left MatchLoader
chassis.turnToPoint(112, -43.25-2.5-0-6.5+2+0.125+.5-.25-.2,800, {}, false);
chassis.moveToPoint(112,-43.25-2.5-0-6.5+2+0.125+.5-.25-.2,1800,{.maxSpeed=40},false);

//Move Outside
chassis.moveToPoint(95, -63,700,{.forwards=false},false);
chassis.turnToPoint(85,-63,400,{.forwards=false},false);
chassis.moveToPoint(85,-63,800,{.forwards=false},false);
//Move To Regular Side
chassis.turnToPoint(0,-63,800,{.forwards=false},false);
chassis.moveToPoint(0,-63,2500,{.forwards=false},false);
//Regular Side Right Inbetween
scraper.set_value(false);
secondStageMotor.move(-127);
chassis.moveToPoint(0,-44.75-2-.5-1,1800,{},false);
//Regular Right Side Long Goal
chassis.turnToPoint(10+6,-44.75-2-.5-1-2,800,{.forwards=false},false);
chassis.moveToPoint(10+6,-44.75-2-.5-1-2, 2000., {.forwards=false}, false);
wings.set_value(false);
scraper.set_value(true);
pros::delay(1800);
wings.set_value(true);
//Regular Side Right Matchloader
scraper.set_value(true);
chassis.turnToPoint(-14.5, -44.75-2-.5+2,650+200,{},false);
chassis.moveToPoint(-14.5,-44.75-2-.5+2,1600,{.maxSpeed=80},false);
//Regular Right Side Long Goal
chassis.turnToPoint(10+6,-44.75-2-.5-1-2,800,{.forwards=false},false);
chassis.moveToPoint(10+6,-44.75-2-.5-1-2, 2000., {.forwards=false}, false);
wings.set_value(false);
scraper.set_value(true);
pros::delay(1800);
wings.set_value(true);
//Parking Zone
scraper.set_value(false);
chassis.moveToPoint(-25,-15,2000,{},false);
chassis.turnToHeading(0,1000,{},false);
float fDDistance = frontDistance.get()/25.4;
chassis.setPose(chassis.getPose().x,-fDDistance,chassis.getPose().theta);
chassis.moveToPose(-25,-1650/25.4,0,3500,{},true);
pros::delay(500);
for(int i = 0; i < 30; i++){
  
  float fDDistance = frontDistance.get()/25.4;
  if(fDDistance<1800 && fDDistance > 1000){
  chassis.setPose(chassis.getPose().x,-fDDistance,chassis.getPose().theta);
  }
  pros::delay(100);
}


//Match Loader Other Side Right Side
// chassis.moveToPoint(94+32,55,1600, {}, false);
// //Back To Scoring
// chassis.moveToPoint(94,54.5,1000,{.forwards=false},false);
// wings.set_value(true);
// scraper.set_value(false);
// pros::delay(1200);
}
void Skills96()
{
  //finding offset using 77 route
// chassis.setPose(0,0,45);
// chassis.moveToPoint(7,7,1000,{},false);
// chassis.turnToPoint(7,0,1000,{},false);
// chassis.moveToPoint(7,0,1000,{},false);
// chassis.turnToHeading(270,1000,{},false);
// float fD = frontDistance.get()*std::cos((chassis.getPose().theta-270)*std::numbers::pi/180)/25.4;
// float rD = rightDistance.get()*std::cos((chassis.getPose().theta-270)*std::numbers::pi/180)/25.4;
// pros::lcd::print(5, "Distance: %f", fD);
// pros::lcd::print(6, "Distance: %f", rD);
// chassis.setPose(0,0,45);
// pros::delay(100000000);
//setup
chassis.setPose(0,0,0);
intakeLift.set_value(false);
// chassis.calibrate(true);
chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
firstStageMotor.move(127);
secondStageMotor.move(-127);
wings.set_value(true);
redirect.set_value(false);


 //Small movement into park zone
 chassis.moveToPoint(0,9,750,{.maxSpeed=70,.minSpeed=70, },false);
 scraper.set_value(true);
 pros::delay(250);
 scraper.set_value(false);
pros::delay(500);
//Move Out Of Park Zone
chassis.moveToPoint(0,38.5,1500,{.maxSpeed=65,.minSpeed=65 },true);
pros::delay(1000);

scraper.set_value(true);
pros::delay(750);
scraper.set_value(false);
pros::delay(500);
chassis.moveToPoint(0, chassis.getPose().y-17, 1000,{.forwards=false,.maxSpeed=40},false);
chassis.moveToPoint(0, chassis.getPose().y+13, 600,{},false);
// pros::delay(1000);
//Reset Position
chassis.turnToHeading(90,500,{},false);
// chassis.moveToPoint(chassis.getPose().x+1, chassis.getPose().y, 300, {}, false);
chassis.moveToPose(chassis.getPose().x-15, chassis.getPose().y, 90,800, {.forwards=false}, false);
scraper.set_value(false);
imu.set_roll(0);
float lD = leftDistance.get()/25.4;
pros::lcd::print(6,"Distance:%d", lD);
//1267/25.4 should corrospend with 0
float yOffset = 1267/25.4;
chassis.setPose(0-20.1395,yOffset-lD+18.4507-2+6,90);

chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);


//setup
firstStageMotor.move(127);
secondStageMotor.move(-127);
wings.set_value(true);
redirect.set_value(false);
//First Left Cluster
chassis.moveToPoint(18.031-2, 34.031, 1400-100,{},false);
firstStageMotor.move(0);
secondStageMotor.move(0);
// chassis.moveToPoint(26.163, 26.163, 700,{.maxSpeed=30},true);

//MidGoal 
//chassis.turnToPoint(39.305+1.5-0.5-2-10-4+10+10-2.5-2+.5-.1+.3+.5+.5-1+.25-0.75+.1+.25,11.021-0.5+16+10-4-10-10+2-.5-.3-1+.25+.5+0.75,550, {.forwards=false},false);
//chassis.moveToPose(39.305+1.5-0.5-2-10-4+10+10-2.5-2+.5-.1+.3+.5+.5-1+.25-0.75+.1+.25, 11.021-0.5+16+10-4-10-10+2-.5-.3-1+.25+.5+0.75,315, 1400,{.forwards=false},false);
chassis.turnToPoint(38.344276, 16.274603,550,{.forwards=false},false);
chassis.moveToPose(38.344276, 16.274603,315,1440,{.forwards=false},false);
redirect.set_value(true);
firstStageMotor.move(127);
secondStageMotor.move(-45);
pros::delay(600);
secondStageMotor.move(-40);
pros::delay(800);
secondStageMotor.move(-35);
pros::delay(1800+2500);

//First Left Inbetween
scraper.set_value(false);
// secondStageMotor.move(-127);
chassis.moveToPoint(0,54.5+0.25-1+.25+1+.75+1+.5-1-1+.1+.5+.1+.2+.2+.15,1200,{},false);
redirect.set_value(false);
firstStageMotor.move(127);
secondStageMotor.move(-127);
//First Left Matchloader
scraper.set_value(true);
chassis.turnToPoint(-14.5+1, 54.5+0.25-1+.25+1+.75+1+.5-1-1+.1+.5+.1+.2+.2+.15,650,{},false);
chassis.moveToPoint(-14.5+1,54.5+0.25-1+.25+1+.75+1+.5-1-1+.1+.5+.1+.2+.2+.15,1600+500,{.maxSpeed=85},false);


//Move Outside
secondStageMotor.move(0);
chassis.moveToPoint(-0, 54.5,600,{.forwards=false},false);
chassis.turnToPoint(14,69,400,{.forwards=false},false);
chassis.moveToPoint(14,69,600,{.forwards=false},false);
//Move to Other Side
// scraper.set_value(false);
chassis.turnToPoint(96,69,500,{.forwards=false},false);
chassis.moveToPoint(96,69,1650,{.forwards=false},false);
//Move In Between
chassis.turnToPoint(96,52.5+0.75+0.5-0.75+0.1-0.5-.25-.4+2.5-1+.5+.6+.1-0.25+.5+.25-.8-.25,800,{},false);//Long Goal
chassis.moveToPoint(96,52.5+0.75+0.5-0.75+0.1-0.5-.25-.4+2.5-1+.5+.6+.1-0.25+.5+.25-.8-.25,900,{},false);//Long Goal
//Long Goal
chassis.turnToPoint(76,52.5+0.75+0.5-0.75+0.1-0.5-.25-.4+2.5-1+.5+.6+.1-0.25+.5+.25-.8-.25,650,{.forwards=false},false);//Long Goal
chassis.moveToPoint(76,52.5+0.75+0.5-0.75+0.1-0.5-.25-.4+2.5-1+.5+.6+.1-0.25+.5+.25-.8-.25,800,{.forwards=false,.maxSpeed=80},false);//Long Goal
//score

secondStageMotor.move(-127);
wings.set_value(false);
scraper.set_value(true);
pros::delay(1200);
//Matchloader
chassis.turnToPoint(111, 52.5+0.75+0.5-0.25-1.5-.5-1+.1+2+.5,800, {}, false);
// chassis.moveToPoint(101, 52.5+0.75+0.5-0.25-1.5-.5-1+.1+2,1000,{}, true);
chassis.moveToPoint(111,52.5+0.75+0.5-0.25-1.5-.5-1+.1+2+.5,1400+100+500,{.maxSpeed=60},true);
pros::delay(600);
wings.set_value(true);
pros::delay(1400);
//Long Goal
chassis.turnToPoint(76,52.5+0.75+0.5-0.75+0.1-0.5-.25-.4+2.5-1+.5+.6+.1-0.25+.5+.25-.8-.25,500,{.forwards=false},false);
chassis.moveToPoint(76,52.5+0.75+0.5-0.75+0.1-0.5-.25-.4+2.5-1+.5+.6+.1-0.25+.5+.25-.8-.25,1000,{.forwards=false,.maxSpeed=80},false);//Long Goal
//Score
secondStageMotor.move(-127);
wings.set_value(false);
scraper.set_value(true);
pros::delay(2000);
//Guessing After This
//Move out A little
secondStageMotor.move(127);
chassis.moveToPoint(90,52.5+0.75+0.5-0.75+0.1-0.5-.25-.125-.5,1000, {}, true);
pros::delay(600);
wings.set_value(true);
secondStageMotor.move(-127);
pros::delay(400);
scraper.set_value(false);

//Move TO Alignment for Parkzone
// chassis.turnToPoint(116,35,1500,{},false);
// chassis.moveToPoint(116,35,1500,{},false);
// chassis.turnToPoint(120,25,1500,{},false);
// chassis.moveToPoint(120,25,1500,{},false);
// chassis.moveToPoint(chassis.getPose().x,35,600,{.forwards=false},false);
// chassis.moveToPoint(120,45,1000,{.maxSpeed=75,.minSpeed=75},false);





//Inbetween Other Left side
chassis.turnToPoint(90,-43.25-2.5-0-6.5+2+0.125+.5-.25-.2+1+.5+1+.25+.2+.2-.75,800,{},false);
chassis.moveToPoint(90,-43.25-2.5-0-6.5+2+0.125+.5-.25-.2+1+.5+1+.25+.2+.2-.75,1900,{},false);
//Left MatchLoader
scraper.set_value(true);
chassis.turnToPoint(112, -43.25-2.5-0-6.5+2+0.125+.5-.25-.2+1+.5+1+.25+.2+.2-.75,800, {}, false);
chassis.moveToPoint(112, -43.25-2.5-0-6.5+2+0.125+.5-.25-.2+1+.5+1+.25+.2+.2-.75,2000,{.maxSpeed=65},false);

//Move Outside
chassis.moveToPoint(100,-43.25-2.5-0-6.5+2+0.125+.5-.25-.2+1+.5+1,600,{.forwards=false},false);
chassis.turnToPoint(87,-63,400,{.forwards=false},false);
chassis.moveToPoint(87,-63,600,{.forwards=false},false);
//Move To Regular Side
chassis.turnToPoint(0,-63,800,{.forwards=false},false);
chassis.moveToPoint(0,-63,2000,{.forwards=false},false);
//Regular Side Right Inbetween
scraper.set_value(false);
secondStageMotor.move(-127);
chassis.turnToPoint(0,-44.75-2-.5-1-2+3-.5-1.5+1.5-.1-.75,1000,{},false);
chassis.moveToPoint(0,-44.75-2-.5-1-2+3-.5-1.5+1.5-.1-.75,1000,{},false);
//Regular Right Side Long Goal
chassis.turnToPoint(12+10,-44.75-2-.5-1-2+3-.5-1.5+1.5-.1-.75,600,{.forwards=false},false);
chassis.moveToPoint(12+10,-44.75-2-.5-1-2+3-.5-1.5+1.5-.1-.75, 800, {.forwards=false}, false);
wings.set_value(false);
scraper.set_value(true);
pros::delay(800);
//Regular Side Right Matchloader
scraper.set_value(true);
chassis.turnToPoint(-14.5, -44.75-2-.5+2-.25,650+200,{},false);
chassis.moveToPoint(-14.5,-44.75-2-.5+2-.25,2000,{.maxSpeed=60},false);
wings.set_value(true);
//Regular Right Side Long Goal
chassis.turnToPoint(12+10,-44.75-2-.5-1-2+3-.5-1.5+1.5-.1-.75,600,{.forwards=false},false);
chassis.moveToPoint(12+10,-44.75-2-.5-1-2+3-.5-1.5+1.5-.1-.75, 1000, {.forwards=false}, false);
wings.set_value(false);
scraper.set_value(false);
pros::delay(1800);
//Parking Zone
chassis.turnToPoint(-2.5, -44.75-2-.5+2-.5,450,{},false);
chassis.moveToPoint(-2.5,-44.75-2-.5+2-.5,800,{},false);
chassis.turnToPoint(-18.5, -24,550,{},false);
chassis.moveToPoint(-18.5,-24,1000,{.maxSpeed=60},false);
chassis.turnToPoint(-18.5, -6,600,{},false);
chassis.moveToPoint(-18.5, -6,900,{.maxSpeed=80,.minSpeed=80},false);

// //Parking Zone
// scraper.set_value(false);
// chassis.moveToPoint(-25,-15,2000,{},false);
// chassis.turnToHeading(0,1000,{},false);
// float fDDistance = frontDistance.get()/25.4;
// chassis.setPose(chassis.getPose().x,-fDDistance,chassis.getPose().theta);
// chassis.moveToPose(-25,-1650/25.4,0,3500,{},true);
// pros::delay(500);
// for(int i = 0; i < 30; i++){
  
//   float fDDistance = frontDistance.get()/25.4;
//   if(fDDistance<1800 && fDDistance > 1000){
//   chassis.setPose(chassis.getPose().x,-fDDistance,chassis.getPose().theta);
//   }
//   pros::delay(100);
// }



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
    case 7:
      Skills96();
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