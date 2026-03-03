#include "robot-config.h"

////////////// DO NOT REMOVE //////////////

// Defines the Brain and Controller
brain Brain;
controller Controller1;

///////////////////////////////////////////

//////////// Drive Train Motors ////////////
// From Back View
motor R1 = motor(PORT12, ratio6_1, true);
motor R2 = motor(PORT13, ratio6_1, true);
motor R3 = motor(PORT14, ratio6_1, true);
motor R4 = motor(PORT15, ratio6_1, true);
motor L1 = motor(PORT17, ratio6_1, false);
motor L2 = motor(PORT18, ratio6_1, false);
motor L3 = motor(PORT19, ratio6_1, false);
motor L4 = motor(PORT20, ratio6_1, false);

///////////////////////////////////////////

//System
motor revolver = motor(PORT10, ratio36_1, false);

motor outtakeLeft = motor(PORT1, ratio18_1, true);
motor outtakeRight = motor(PORT11, ratio18_1, false);
motor_group outtake = motor_group(outtakeLeft, outtakeRight);

motor intakeLeft = motor(PORT2, ratio36_1, true);
motor intakeRight = motor(PORT9, ratio36_1, false);
motor_group intake = motor_group(intakeLeft, intakeRight);

//////////// Odometry Sensors ////////////
//TODO: CHANGE THE NAMES OF THESE
rotation rotationH = rotation(PORT4); // Horizontal
rotation rotationF = rotation(PORT5);  // Forward

/////////////////////////////////////////

//////////// Inertial Sensors ////////////

inertial inertial1 = inertial(PORT8);
/////////////////////////////////////////

led liftR = led(Brain.ThreeWirePort.A);
led liftL = led(Brain.ThreeWirePort.B);
led matchLoader = led(Brain.ThreeWirePort.C);
led extendo = led(Brain.ThreeWirePort.G);

/////////////////////////////////////

optical backColorSensor = optical(PORT7);
optical middleColorSensor = optical(PORT6);
optical frontColorSensor = optical(PORT16);

void setDriveTrainConstants() {
  // Set the Drive PID values for the DriveTrain
    chassis.setDriveTrainConstants(
        2.481, // Kp - Proportion Constant
        0.0, // Ki - Integral Constant
        10.0, // Kd - Derivative Constant
        1.0, // Settle Error
        100 // Time to Settle
    );

    // Set the Turn PID values for the DriveTrain
    chassis.setTurnConstants(
        0.504,        // Kp - Proportion Constant
        0.0,         // Ki - Integral Constant
        4.05,       // Kd - Derivative Constant 
        3.0,       // Settle Error
        200      // Time to Settle
    );
}

void vexcodeInit(void) {
  setDriveTrainConstants();
  enum preAutonStates{START_SCREEN = 0, SELECTION_SCREEN = 1};
  int currentScreen = START_SCREEN;
  int lastPressed = 0;

  // Calibrates/Resets the Brains sensors before the competition
  inertial1.calibrate();
  rotationH.resetPosition();
  rotationF.resetPosition();
}