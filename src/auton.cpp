#include "vex.h"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * driveDistance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void defaultConstants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.setDriveTrainConstants(10, 1.5, 0, 10, 0);
  chassis.setHeadingConstants(6, .4, 0, 1, 0);
  chassis.setTurnConstants(12, .4, .03, 3, 15);
  chassis.setSwingConstants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.setDriveExitConditions(1.5, 300, 5000);
  chassis.setTurnExitConditions(1, 300, 3000);
  chassis.setSwingExitConditions(1, 300, 3000);
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like driveToPoint(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  defaultConstants();
  chassis.headingMaxVoltage = 10;
  chassis.driveMaxVoltage = 8;
  chassis.driveSettleError = 3;
  chassis.boomerangLead = 0.5;
  chassis.driveMinVoltage = 1.45;
}

/**
 * The expected behavior is to return to the start position.
 */

void driveTest(){
  chassis.driveDistance(6);
  chassis.driveDistance(12);
  chassis.driveDistance(18);
  chassis.driveDistance(-36);
}

/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */

void turnTest(){
  chassis.turnToAngle(5);
  chassis.turnToAngle(30);
  chassis.turnToAngle(90);
  chassis.turnToAngle(225);
  chassis.turnToAngle(0);
}

/**
 * Should swing in a fun S shape.
 */

void swingTest(){
  chassis.leftSwingToAngle(90);
  chassis.rightSwingToAngle(0);
}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void fullTest(){
  chassis.driveDistance(24);
  chassis.turnToAngle(-45);
  chassis.driveDistance(-36);
  chassis.rightSwingToAngle(-90);
  chassis.driveDistance(24);
  chassis.turnToAngle(0);
}

/**
 * Doesn't drive the robot, but just prints coordinates to the Brain screen 
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

void odomTest(){
  chassis.setCoordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.getXPosition());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.getYPosition());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.getAbsoluteHeading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.getForwardTrackerPosition());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.getSidewaysTrackerPosition());
    task::sleep(20);
  }
}

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */

void tankOdomTest(){
  odom_constants();
  chassis.setCoordinates(0, 0, 0);
  chassis.turnToPoint(24, 24);
  chassis.driveToPoint(24,24);
  chassis.driveToPoint(-24,-24);
  chassis.turnToAngle(0);
}

/**
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */

void holonomicOdomTest(){
  odom_constants();
  chassis.setCoordinates(0, 0, 0);
  chassis.holonomicDriveToPose(0, 18, 90);
  chassis.holonomicDriveToPose(18, 0, 180);
  chassis.holonomicDriveToPose(0, 18, 270);
  chassis.holonomicDriveToPose(0, 0, 0);
}

void auton1()
{
  chassis.setCoordinates(-47.591, -13.958, 325);
  //chassis.turnToPoint(-20, -22);
  chassis.driveToPoint(-56.754,-47.375);
  chassis.turnToAngle(275);
}