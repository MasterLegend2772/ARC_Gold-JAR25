#pragma once

#include "vex.h"

using namespace vex;

extern brain Brain;
extern controller Controller1;

extern motor R1;
extern motor R2;
extern motor R3;
extern motor R4;
extern motor L1;
extern motor L2;
extern motor L3;
extern motor L4;

extern motor_group rightDrive;
extern motor_group leftDrive;

extern motor revolver;
extern motor outtakeLeft;
extern motor outtakeRight;
extern motor_group outtake;
extern motor intakeLeft;
extern motor intakeRight;
extern motor_group intake;

extern rotation rotationH;
extern rotation rotationF;

extern inertial inertial1;

extern led liftR;
extern led liftL;
extern led matchLoader;
extern led extendo;

extern distance backDistanceSensor;
extern distance middleDistanceSensor;
extern distance frontDistanceSensor;

void  vexcodeInit(void);