/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Tuesday, December 11, 2007, Birth of a Legend                                          */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// rotation1            rotation      5               
// expansion            digital_out   B               
// intake               motor         15              
// cata                 motor         21              
// Controller1          controller                    
// booster              digital_out   A               
// lowerEX              digital_out   C               
// inertial1            inertial      14              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

// Brain and Controller
brain Brain;

controller Controller1 = controller(primary);
bool RemoteControlCodeEnabled = true;

// Left Drive Motors
motor left1 = motor(PORT15, ratio6_1, true);
motor left2 = motor(PORT11, ratio6_1, true);
motor left3 = motor(PORT12, ratio6_1, true);
motor_group LeftDrive = motor_group(left1, left2, left3);

// Right Drive Motors
motor right1 = motor(PORT2, ratio6_1, false);
motor right2 = motor(PORT14, ratio6_1, false);
motor right3 = motor(PORT17, ratio6_1, false);
motor_group RightDrive = motor_group(right1, right2, right3);

// Drivetrain
drivetrain Drivetrain = drivetrain(LeftDrive, RightDrive, 3.25, 11.5, 11.5, inches);

double Wheel = 3.25;
double WB = 11.5;
double EncPerIn = 360 / (Wheel * M_PI);

// Single motors
rotation rotation1 = rotation(PORT5, false);
motor intake = motor(PORT16, ratio6_1, false);
motor cata = motor(PORT21, ratio18_1, false);

// Sensors
digital_out booster = digital_out(Brain.ThreeWirePort.A);
digital_out lowerEX = digital_out(Brain.ThreeWirePort.C);
inertial inertial1 = inertial(PORT14);
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

const double WHEEL_SIZE = 3.25;  // inches
//const double TRACK_WIDTH = 11.5;  // inches

double xPos = 0.0;  // inches
double yPos = 0.0;  // inches
double heading = 90.0;  // degrees

double prevLeftEncoder = 0.0;
double prevRightEncoder = 0.0;
double prevHeading = 0.0;

void updateOdom() {
  double leftEncoder = LeftDrive.position(degrees);
  double rightEncoder = RightDrive.position(degrees);
  double newHeading = inertial1.heading(degrees);

  double deltaLeft = (leftEncoder - prevLeftEncoder) / 360.0 * WHEEL_SIZE * M_PI;
  double deltaRight = (rightEncoder - prevRightEncoder) / 360.0 * WHEEL_SIZE * M_PI;
  
  double deltaDistance = (deltaLeft + deltaRight) / 2.0;
  double deltaHeading = (newHeading - prevHeading);

  double cosH = cos(deltaHeading * M_PI / 180.0);
  double sinH = sin(deltaHeading * M_PI / 180.0);
 
  double dx = deltaDistance * sinH * -1;
  double dy = deltaDistance * cosH;
  
  xPos += dx;
  yPos += dy;
  heading = newHeading;
  
  prevLeftEncoder = leftEncoder;
  prevRightEncoder = rightEncoder;
  prevHeading = newHeading;

}

bool enableDrivePID;
bool halfSpeed;

double kP = 0.05;
double kI = 0.0;
double kD = 0.2;
double turnkP = 0.05;
double turnkI = 0.0;
double turnkD = 0.1;

bool resetDriveSensors;
double targetValue;
double targetTurnValue;

int drivePID() {

  double error;
  double prevError = 0.0;
  double derivative;
  double totalError = 0.0;
  double turnError;
  double turnPrevError = 0.0;
  double turnDerivative;
  double turnTotalError = 0.0;

  while (enableDrivePID) {

    if (resetDriveSensors) {
      resetDriveSensors = false;
      LeftDrive.setPosition(0, degrees);
      RightDrive.setPosition(0, degrees);
    }

    double leftMotorPosition = LeftDrive.position(degrees);
    double rightMotorPosition = RightDrive.position(degrees);

    // Lateral movement PID
    double averagePosition = (leftMotorPosition + rightMotorPosition) / 2;
    error = targetValue - averagePosition;
    derivative = error - prevError;
    totalError += error;
    double lateralMotorPower = error * kP + derivative * kD + totalError * kI;

    // Turning movement PID
    double turnDifference = leftMotorPosition - rightMotorPosition;
    turnError = targetTurnValue - turnDifference;
    turnDerivative = turnError - turnPrevError;
    turnTotalError += turnError;
    double turnMotorPower =
        turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI;

    LeftDrive.spin(forward, lateralMotorPower/(halfSpeed ? 20 : 1) + turnMotorPower,
                   voltageUnits::volt);
    RightDrive.spin(forward, lateralMotorPower*0.8/(halfSpeed ? 20 : 1) - turnMotorPower,
                    voltageUnits::volt);

    prevError = error;
    turnPrevError = turnError;
    updateOdom();

    vex::task::sleep(20);
  }
  return 1;
}

void goStraight(double distIn) { //inches
  resetDriveSensors = true;
  targetValue = distIn * EncPerIn;
  targetTurnValue = 0.0;
  wait(distIn < 0 ? -distIn / 25.0 : distIn / 25.0, sec);
}

void turn(double degrees) {
  resetDriveSensors = true;
  targetValue = 0.0;
  targetTurnValue = degrees * -12.5;
  wait(0.5, sec);
}

void turnTo(double setRotation) {
  double delta = setRotation - heading;
  turn(delta);
}

void roller() {
  intake.resetPosition();
  // intake.resetRotation();
  intake.spinToPosition(-1050.0, deg, true);
  wait(0.5, sec);
}

int shoot() {
  cata.spin(forward);
  wait(0.75, sec);
  waitUntil(rotation1.position(degrees) > 70);
  wait(0.0, sec);
  cata.stop();

  return 1;
}

void goTo(double targetX, double targetY) {
  // Calculate the direction and distance to the target location
  double deltaX = targetX - xPos;
  double deltaY = targetY - yPos;
  double distance = sqrt(deltaX * deltaX + deltaY * deltaY);
  double angle = atan2(deltaY, deltaX) * 180.0 / M_PI;
  
  // Turn the robot to face the target location
  turnTo(angle);
  wait(0.5, sec);
  
  // Move the robot forward to the target location
  goStraight(distance);
  
  // Displays coord values on controller screen
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("targetPos: " + "(" + targetX + ", " + targetY + ")");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("actualPos: " + "(" + xPos + ", " + yPos + ")");
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("--------------------------------------------------");
    
}

void goToReverse(double targetX, double targetY) {
  // Calculate the direction and distance to the target location
  double deltaX = targetX - xPos;
  double deltaY = targetY - yPos;
  double distance = sqrt(deltaX * deltaX + deltaY * deltaY);
  double angle = atan2(deltaY, deltaX) * 180.0 / M_PI - 90.0;
  
  // Turn the robot to face the target location
  turnTo(angle);
  wait(0.5, sec);
  
  // Move the robot forward to the target location
  goStraight(distance * -1);
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  //vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  LeftDrive.setVelocity(100, percent);
  LeftDrive.setStopping(coast);
  RightDrive.setVelocity(100, percent);
  RightDrive.setStopping(coast);
  intake.setVelocity(100, percent);
  intake.setStopping(coast);
  cata.setVelocity(65, percent);
  cata.setStopping(hold);
  cata.setMaxTorque(100, percent);

  rotation1.setPosition(0.0, degrees);
  inertial1.setHeading(0, degrees);

  Brain.Screen.setFont(propXXL);
  //Brain.Screen.print("Leo is cool");
  //Brain.Screen.setCursor(2, 1);
  //Brain.Screen.print("200PTautonwhen :)");
  //Brain.Screen.setCursor(3, 1);
  //Brain.Screen.print("Leo is smarter than");
  //Brain.Screen.setCursor(4, 1);
  //Brain.Screen.print("Gary");
  Brain.Screen.setCursor(2, 8);
  Brain.Screen.print("10K");
  Brain.Screen.setCursor(3, 4);
  Brain.Screen.print("#cata-strophic");
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  xPos = 0.0;  // inches
  yPos = 0.0;  // inches
  //heading = 90.0;  // degrees

  prevLeftEncoder = 0.0;
  prevRightEncoder = 0.0;

  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print(xPos);
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print(yPos);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print(heading);

  vex::task PID(drivePID);
  enableDrivePID = true;
  resetDriveSensors = true;

  goTo(60, 0);
  //goTo(0, 60);
  //goTo(60, 60);
  
  //goTo(0, 144);

/*
  vex::task t(shoot);
  goStraight(3);
  roller();
  goToReverse(0, -5);
  goTo(5, -8);
  turnTo(-90);
  goTo(15, -8);
  roller();
  goToReverse(5, 8);
  goToReverse(5, -40);
  */



  
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

bool StopLeft = false;
bool StopRight = false;
bool StopIntake = false;
bool StopForwardCatapult = false;


// "when started" hat block
int SecondaryPreAuton() {
  LeftDrive.setVelocity(100.0, percent);
  LeftDrive.setStopping(coast);
  RightDrive.setVelocity(100.0, percent);
  RightDrive.setStopping(coast);
  intake.setVelocity(100.0, percent);
  intake.setStopping(coast);
  cata.setVelocity(50.0, percent);
  cata.setStopping(hold);
  cata.setMaxTorque(100, percent);
  rotation1.setPosition(0.0, degrees);
  inertial1.setHeading(90.0, degrees);
  return 0;
}

// "when Controller1 ButtonL1 pressed" hat block
void intake_in() {
  intake.spin(forward);
  wait(1.0, seconds);
  waitUntil((Controller1.ButtonL1.pressing() || Controller1.ButtonL2.pressing()));
  intake.stop();
  wait(0.25, seconds);
  /*
  if(StopIntake) {
    StopIntake = false;
  } else {
    StopIntake = true;
  }
  */
}

// "when Controller1 ButtonL2 pressed" hat block
void intake_out() {
  intake.spin(reverse);
}

// "when Controller1 ButtonL2 released" hat block
void intake_stop() {
  intake.stop();
}

// "when Controller1 ButtonR1 pressed" hat block
void cata_fire() {
  cata.spin(forward);
  wait(0.75, seconds);
  waitUntil(rotation1.position(degrees) > 104);
  wait(0.00, seconds);
  cata.stop();  
}

// "when Controller1 ButtonR2 pressed" hat block
void cata_reverse() {
  cata.spin(reverse);
}

// "when Controller1 ButtonR2 released" hat block
void cata_stop_reverse() {
  cata.stop();
}

// "when Controller1 ButtonUp pressed" hat block
void cata_forward() {
  cata.spin(forward);
}

// "when Controller1 ButtonUp released" hat block
void cata_stop_forward() {
  cata.stop();
}

void onevent_Controller1ButtonX_pressed_0() {
  booster.set(true);
}

void onevent_Controller1ButtonY_pressed_0() {
  booster.set(false);
}

void onevent_Controller1ButtonLeft_pressed_0() {
  lowerEX.set(true);
}

void onevent_Controller1ButtonDown_pressed_0() {
  lowerEX.set(false);
}

void usercontrol(void) {
  // User control code here, inside the loop
  enableDrivePID = false;
  
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    float LeftSpeed;
    float RightSpeed;

    if (Controller1.Axis3.position() > 0) {
      LeftSpeed = (Controller1.Axis3.position() * 0.1) * (Controller1.Axis3.position() * 0.1);
    } else {
      LeftSpeed = (Controller1.Axis3.position() * 0.1) * (Controller1.Axis3.position() * 0.1) * -1;
    }

    if (Controller1.Axis2.position() > 0) {
      RightSpeed = (Controller1.Axis2.position() * 0.1) * (Controller1.Axis2.position() * 0.1);
    } else {
      RightSpeed = (Controller1.Axis2.position() * 0.1) * (Controller1.Axis2.position() * 0.1) * -1;
    }

    if(LeftSpeed > -10 && LeftSpeed < 10) {
      if(StopLeft) {
        LeftDrive.stop();
        StopLeft = false;
      }
    } else {
      StopLeft = true;
    }

    if(RightSpeed > -10 && RightSpeed < 10) {
      if(StopRight) {
        RightDrive.stop();
        StopRight = false;
      }
    } else {
      StopRight = true;
    }

    if(StopLeft) {
      LeftDrive.setVelocity(LeftSpeed, percent);
      LeftDrive.spin(forward);
    }

    if(StopRight) {
      RightDrive.setVelocity(RightSpeed, percent);
      RightDrive.spin(forward);
    }
    
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
  
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  
  Controller1.ButtonL1.pressed(intake_in);
  Controller1.ButtonL2.pressed(intake_out);
  Controller1.ButtonL2.released(intake_stop);
  Controller1.ButtonR1.pressed(cata_fire);
  Controller1.ButtonR2.pressed(cata_reverse);
  Controller1.ButtonR2.released(cata_stop_reverse);
  Controller1.ButtonUp.pressed(cata_forward);
  Controller1.ButtonUp.released(cata_stop_forward);
  Controller1.ButtonX.pressed(onevent_Controller1ButtonX_pressed_0);
  Controller1.ButtonY.pressed(onevent_Controller1ButtonY_pressed_0);
  Controller1.ButtonLeft.pressed(onevent_Controller1ButtonLeft_pressed_0);
  Controller1.ButtonDown.pressed(onevent_Controller1ButtonDown_pressed_0);

  
  wait(15, msec);
  // post event registration

  // wait for rotation sensor to fully initialize
  wait(30, msec);


  // Run the pre-autonomous function.
  pre_auton();
  SecondaryPreAuton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    //Controller1.Screen.print("Timer: %8", Brain.Timer.value());
    wait(100, msec);
  }
}