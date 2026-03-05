#include "vex.h"
#include "auton.h"

using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                        ARC Gold  Template  Config                         */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/

//Used for color sort
  // int blueTeam = 1;
  // int redTeam = 2;
  int teamColor = 1;

///////////////////////// Functions & Prototypes /////////////////////////
void toggleDriveSpeed();
void SetSlot();
void TopSlotMajorityEnemy(int);
void transferArrayInfo();
void AutonSkills_Left(); // Autonnomous Route to Run
void outTake();
void rotateRevolver();
void usercontrol();

bool armUp = false;
bool isInAuton = false;

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
ZERO_TRACKER_ODOM,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Drivetrain Motors:
motor_group(L1, L2, L3, L4),

//Right Drivetrain Motors:
motor_group(R1, R2, R3, R4),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT8,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
2.75,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
1.0,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360.00,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
PORT5,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
1.875,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
-2.75, // Back::  1.5 inches

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
PORT4,

//Sideways tracker diameter (reverse to make the direction switch):
1.875,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
1.75 // Back:: 3.5 inches

);

int currentAutonSelection = 5;
bool autoStarted = false;

/**
 * Function before autonomous. It prints the current auton number on the screen
 * and tapping the screen cycles the selected auton by 1. Add anything else you
 * may need, like resetting pneumatic components. You can rename these autons to
 * be more descriptive, if you like.
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  defaultConstants();

  while (!autoStarted){

    Brain.Screen.clearScreen();
    Brain.Screen.printAt(1, 10, "ARC_Gold Template v0.0.1");
    Brain.Screen.printAt(1, 40, "Battery Percentage: ");
    Brain.Screen.printAt(1, 60, "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(8, 20, "Chassis Heading Reading: ");
    Brain.Screen.printAt(8, 60, "%f", chassis.getAbsoluteHeading());
    Brain.Screen.printAt(15, 20, "Selected Auton:");
    switch(currentAutonSelection){
      
      case 0:
        Brain.Screen.printAt(7, 50, "Auton 1");
        break;
      case 1:
        Brain.Screen.printAt(7, 50, "Auton 2");
        break;
      case 2:
        Brain.Screen.printAt(7, 50, "Auton 3");
        break;
      case 3:
        Brain.Screen.printAt(7, 50, "Auton 4");
        break;
      case 4:
        Brain.Screen.printAt(7, 50, "Auton 5");
        break;
      case 5:
        Brain.Screen.printAt(7, 50, "Auton 6");
        break;
      case 6:
        Brain.Screen.printAt(7, 50, "Auton 7");
        break;
      case 7:
        Brain.Screen.printAt(7, 50, "Auton 8");
        break;
    }
    if (Brain.Screen.pressing()){
      while (Brain.Screen.pressing()) {}
      currentAutonSelection++;
    } else if (currentAutonSelection == 8){
      currentAutonSelection = 0;
    }
    task::sleep(10);
  }
}

/**
 * Auton function, which runs the selected auton. Case 0 is the default,
 * and will run in the brain screen goes untouched during preauton. Replace
 * driveTest(), for example, with your own auton function you created in
 * autons.cpp and declared in autons.h.
 */

void autonomous(void) {
  autoStarted = true;
  
  switch(currentAutonSelection){ 
    case 0:
      driveTest();
      break;
    case 1:         
      driveTest();
      break;
    case 2:
      turnTest();
      break;
    case 3:
      swingTest();
      break;
    case 4:
      fullTest();
      break;
    case 5:
      odomTest();
      break;
    case 6:
      tankOdomTest();
      break;
    case 7:
      holonomicOdomTest();
      break;
 }
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

// Additional Driver Control Functions Here //

/******************************************************************
 * Function: toggleLift()
 * 
 * Purpose: Manual toggle for Lift
*******************************************************************/

void toggleLift() {
  liftL.set(!liftL.value());
  liftR.set(!liftR.value());
}


/******************************************************************
 * Function: outTake()
 * 
 * Purpose: Shoots & Outtakes TOP Slot Blocks
*******************************************************************/
void outTake() { 
  if (!revolver.isSpinning()) {
    armUp = true;
    outtake.stop(coast);
    outtake.setVelocity(100, percent);
    outtake.spinToPosition(100, degrees, true);
    outtake.spinFor(reverse, 0.75, sec);
    outtake.stop(hold);
    armUp = false;
  }
}


/******************************************************************
 * Function: moveIntake()
 * 
 * Purpose: Rotates Revolver by ONE Slot
*******************************************************************/
void moveIntake() {
  if ((!revolver.isSpinning()) && ((liftL.value() == false) && (liftR.value() == false))) {
    intake.spin(forward, 12, volt);
  }
}


/******************************************************************
 * Function: moveSlot()
 * 
 * Purpose: Reverse Rotate Geneva while button is pressed
*******************************************************************/
void moveSlot() {
    outtake.setVelocity(100, percent);
    outtake.spinFor(reverse, 0.1, sec);
    outtake.stop(hold);
    revolver.setBrake(coast);

    PID revolverPID(0.0, 0.66, 0.0, 2.0, 0.0, 10.0, 100.0, 2000.0);
    float desiredPos = 360 + revolver.position(degrees);

    while (!revolverPID.isSettled()) {
      Brain.Screen.setCursor(1,1);
      Brain.Screen.print("Is Revolving");
      float currentPos = revolver.position(degrees);

      float output = revolverPID.compute(desiredPos - currentPos);
      output = clamp(output, -12, 12);

      revolver.spin(forward, output, volt);
      wait(10, msec);
      Brain.Screen.clearScreen();
    }
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Is Revolved");

    revolver.spin(forward, 0, volt);
    revolver.setBrake(hold);
    outtake.stop(coast);
}


/******************************************************************
 * Function: unloadAll()
 * 
 * Purpose: Function to Unload ALL Blocks out of Revolver
*******************************************************************/
void unloadAll() {
  for (int i = 0; i <6; i++) {
    outTake();
    moveSlot();
    waitUntil(!revolver.isSpinning());
  }
}


/******************************************************************
 * Function: isSlotFull()
 * 
 * Purpose: Checks if Revolver Slot is Full 
 *          via Distance Sensors
*******************************************************************/
bool isSlotFull() {
  //Says, "If frontSensor is between 0 and 20(red), or frontSensor is between 170 and 200(blue)"
  //AND "MiddleSensor is red or blue"
  //AND "BackSensor is red or blue"
  //Then return that Slot is full (true)

  if ((frontDistanceSensor.objectDistance(inches) <= 2) && (middleDistanceSensor.objectDistance(inches) <= 2) && (backDistanceSensor.objectDistance(inches) <= 2)) {
      Brain.Screen.setCursor(1,1);
      Brain.Screen.print("Is Full");
      return true;
    } else return false;
} 


/******************************************************************
 * Function: fixGeneva()
 * 
 * Purpose: Reverses rotaion of Geneva for however 
 *          long button held
*******************************************************************/
void fixGeneva() {
  if (!revolver.isSpinning())
    revolver.spin(reverse, 8, volt);
}


/**/
void setBrake() {
  R1.setBrake(coast);
  R2.setBrake(coast);
  R3.setBrake(coast);
  R4.setBrake(coast);
  L1.setBrake(coast);
  L2.setBrake(coast);
  L3.setBrake(coast);
  L4.setBrake(coast);
}

// Check Canister
bool isSlotFull();



/* User Control Period Function to initialize ALL Functions && Variables
 * during manual Driving Control. */
void usercontrol() {
  // User control code here

  // Variables & User Control Resets (if necessary)
  isInAuton = true;

  setBrake();

  extendo.set(true);

  outtake.spin(reverse, 9, volt);
  wait(0.1, sec);
  outtake.spin(reverse, 0, volt);
  outtake.stop(hold);

  Brain.Screen.clearScreen();

  Controller1.ButtonR1.pressed(outTake);
  // FREE:: Controller1.ButtonR2.pressed();

  Controller1.ButtonL1.pressed(moveIntake);

  Controller1.ButtonLeft.pressed(fixGeneva);

  bool liftToggle = false;
  bool matchLoadToggle = false;

  // User Control Code inside loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

  // User Programs || Functions || Controls Here

    //To stop geneva fixer from spinning when button is released
    if (!Controller1.ButtonLeft.pressing()) {
      revolver.spin(forward, 0, volt);
    }

    // Moves Revolver Slot
    if (Controller1.ButtonB.pressing() && !revolver.isSpinning()) {
      if (!armUp) {
        moveSlot();
      }
    }

    // Automatic Rotation
    if ((Controller1.ButtonL1.pressing() || Controller1.ButtonL2.pressing()) && isSlotFull()) {
      if (!armUp) {
        moveSlot();
      }
    }

    // Lift Toggle (Rise || Fall)
    if (Controller1.ButtonRight.pressing() && !liftToggle) {
      toggleLift();
      liftToggle = true;
    } else if (!Controller1.ButtonRight.pressing() && liftToggle) {
      liftToggle = false;
    }

    // Matchload Toggle (Rise || Fall)
    if (Controller1.ButtonL2.pressing() && !matchLoadToggle) {
      matchLoader.set(!matchLoader.value());
      matchLoadToggle = true;
    } else if (!Controller1.ButtonL2.pressing() && matchLoadToggle) {
      matchLoadToggle = false;
    }

    // Stops Intake Spin (Forward && Reverse)
    if (!Controller1.ButtonL1.pressing() && !Controller1.ButtonL2.pressing() && !Controller1.ButtonR2.pressing()) {
      intake.spin(reverse, 0, volt);
    }

    // Replace this line with [chassis.controlTank()] for tank drive 
    // or [chassis.controlHolonomic()] for holonomic drive.
    chassis.controlArcade(); // !!! Control Tank for Coby !!!

    wait(20, msec); 
    // Sleep the task for a short amount of time to
    // prevent wasted resources.
  }
}

// Main will set up the competition functions and callbacks.
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
