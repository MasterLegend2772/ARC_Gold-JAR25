#pragma once
#include "vex.h"

using namespace vex;

enum driveSetup {ZERO_TRACKER_NO_ODOM, ZERO_TRACKER_ODOM, TANK_ONE_FORWARD_ENCODER, TANK_ONE_FORWARD_ROTATION, 
TANK_ONE_SIDEWAYS_ENCODER, TANK_ONE_SIDEWAYS_ROTATION, TANK_TWO_ENCODER, TANK_TWO_ROTATION, 
HOLONOMIC_TWO_ENCODER, HOLONOMIC_TWO_ROTATION};

/**
 * Drive class supporting tank and holo drive, with or without odom.
 * Eight flavors of odom and six custom motion algorithms.
 */

class Drive
{
private:
  float wheelDiameter;
  float wheelRatio;
  float gyroScale;
  float driveInToDegRatio;
  float ForwardTrackerCenterDistance;
  float ForwardTrackerDiameter;
  float ForwardTrackerInToDegRatio;
  float SidewaysTrackerCenterDistance;
  float SidewaysTrackerDiameter;
  float SidewaysTrackerInToDegRatio;
  vex:: triport ThreeWire = vex::triport(vex::PORT22);

public: 
  driveSetup driveSetup = ZERO_TRACKER_NO_ODOM;
  motor_group DriveL;
  motor_group DriveR;
  inertial Gyro;
  motor DriveLF;
  motor DriveRF;
  motor DriveLB;
  motor DriveRB;
  rotation RForwardTracker;
  rotation RSidewaysTracker;
  encoder EForwardTracker;
  encoder ESidewaysTracker;

  float turnMaxVoltage;
  float turnKp;
  float turnKi;
  float turnKd;
  float turnStarti;

  float turnSettleError;
  float turnSettleTime;
  float turnTimeout;

  float driveMinVoltage;
  float driveMaxVoltage;
  float driveKp;
  float driveKi;
  float driveKd;
  float driveStarti;

  float driveSettleError;
  float driveSettleTime;
  float driveTimeout;

  float headingMaxVoltage;
  float headingKp;
  float headingKi;
  float headingKd;
  float headingStarti;

  float swingMaxVoltage;
  float swingKp;
  float swingKi;
  float swingKd;
  float swingStarti;

  float swingSettleError;
  float swingSettleTime;
  float swingTimeout;

  float boomerangLead;
  float boomerang_setback;

  Drive(enum::driveSetup driveSetup, motor_group DriveL, motor_group DriveR, int gyro_port, float wheelDiameter, float wheelRatio, float gyroScale, int DriveLF_port, int DriveRF_port, int DriveLB_port, int DriveRB_port, int ForwardTracker_port, float ForwardTrackerDiameter, float ForwardTrackerCenterDistance, int SidewaysTracker_port, float SidewaysTrackerDiameter, float SidewaysTrackerCenterDistance);

  void driveWithVoltage(float leftVoltage, float rightVoltage);

  float getAbsoluteHeading();

  float getLeftPositionIn();

  float getRightPositionIn();

  void setTurnConstants(float turnMaxVoltage, float turnKp, float turnKi, float turnKd, float turnStarti); 
  void setDriveTrainConstants(float driveMaxVoltage, float driveKp, float driveKi, float driveKd, float driveStarti);
  void setHeadingConstants(float headingMaxVoltage, float headingKp, float headingKi, float headingKd, float headingStarti);
  void setSwingConstants(float swingMaxVoltage, float swingKp, float swingKi, float swingKd, float swingStarti);

  void setTurnExitConditions(float turnSettleError, float turnSettleTime, float turnTimeout);
  void setDriveExitConditions(float driveSettleError, float driveSettleTime, float driveTimeout);
  void setSwingExitConditions(float swingSettleError, float swingSettleTime, float swingTimeout);

  void turnToAngle(float angle);
  void turnToAngle(float angle, float turnMaxVoltage);
  void turnToAngle(float angle, float turnMaxVoltage, float turnSettleError, float turnSettleTime, float turnTimeout);
  void turnToAngle(float angle, float turnMaxVoltage, float turnSettleError, float turnSettleTime, float turnTimeout, float turnKp, float turnKi, float turnKd, float turnStarti);

  void driveDistance(float distance);
  void driveDistance(float distance, float heading);
  void driveDistance(float distance, float heading, float driveMaxVoltage, float headingMaxVoltage);
  void driveDistance(float distance, float heading, float driveMaxVoltage, float headingMaxVoltage, float driveSettleError, float driveSettleTime, float driveTimeout);
  void driveDistance(float distance, float heading, float driveMaxVoltage, float headingMaxVoltage, float driveSettleError, float driveSettleTime, float driveTimeout, float driveKp, float driveKi, float driveKd, float driveStarti, float headingKp, float headingKi, float headingKd, float headingStarti);

  void leftSwingToAngle(float angle);
  void leftSwingToAngle(float angle, float swingMaxVoltage, float swingSettleError, float swingSettleTime, float swingTimeout, float swingKp, float swingKi, float swingKd, float swingStarti);
  void rightSwingToAngle(float angle);
  void rightSwingToAngle(float angle, float swingMaxVoltage, float swingSettleError, float swingSettleTime, float swingTimeout, float swingKp, float swingKi, float swingKd, float swingStarti);
  
  Odom odom;
  float getForwardTrackerPosition();
  float getSidewaysTrackerPosition();
  void setCoordinates(float xPosition, float yPosition, float orientationDeg);
  void setHeading(float orientationDeg);
  void positionTrack();
  static int positionTrackTask();
  vex::task odomTask;
  float getXPosition();
  float getYPosition();

  void drive_stop(vex::brakeType mode);

  void driveToPoint(float xPosition, float yPosition);
  void driveToPoint(float xPosition, float yPosition, float driveMinVoltage, float driveMaxVoltage, float headingMaxVoltage);
  void driveToPoint(float xPosition, float yPosition, float driveMinVoltage, float driveMaxVoltage, float headingMaxVoltage, float driveSettleError, float driveSettleTime, float driveTimeout);
  void driveToPoint(float xPosition, float yPosition, float driveMinVoltage, float driveMaxVoltage, float headingMaxVoltage, float driveSettleError, float driveSettleTime, float driveTimeout, float driveKp, float driveKi, float driveKd, float driveStarti, float headingKp, float headingKi, float headingKd, float headingStarti);
  
  void driveToPose(float xPosition, float yPosition, float angle);
  void driveToPose(float xPosition, float yPosition, float angle, float lead, float setback, float driveMinVoltage);
  void driveToPose(float xPosition, float yPosition, float angle, float lead, float setback, float driveMinVoltage, float driveMaxVoltage, float headingMaxVoltage);
  void driveToPose(float xPosition, float yPosition, float angle, float lead, float setback, float driveMinVoltage, float driveMaxVoltage, float headingMaxVoltage, float driveSettleError, float driveSettleTime, float driveTimeout);
  void driveToPose(float xPosition, float yPosition, float angle, float lead, float setback, float driveMinVoltage, float driveMaxVoltage, float headingMaxVoltage, float driveSettleError, float driveSettleTime, float driveTimeout, float driveKp, float driveKi, float driveKd, float driveStarti, float headingKp, float headingKi, float headingKd, float headingStarti);
  
  void turnToPoint(float xPosition, float yPosition);
  void turnToPoint(float xPosition, float yPosition, float extraAngleDeg);
  void turnToPoint(float xPosition, float yPosition, float extraAngleDeg, float turnMaxVoltage, float turnSettleError, float turnSettleTime, float turnTimeout);
  void turnToPoint(float xPosition, float yPosition, float extraAngleDeg, float turnMaxVoltage, float turnSettleError, float turnSettleTime, float turnTimeout, float turnKp, float turnKi, float turnKd, float turnStarti);
  
  void holonomicDriveToPose(float xPosition, float yPosition);
  void holonomicDriveToPose(float xPosition, float yPosition, float angle);
  void holonomicDriveToPose(float xPosition, float yPosition, float angle, float driveMaxVoltage, float headingMaxVoltage);
  void holonomicDriveToPose(float xPosition, float yPosition, float angle, float driveMaxVoltage, float headingMaxVoltage, float driveSettleError, float driveSettleTime, float driveTimeout);
  void holonomicDriveToPose(float xPosition, float yPosition, float angle, float driveMaxVoltage, float headingMaxVoltage, float driveSettleError, float driveSettleTime, float driveTimeout, float driveKp, float driveKi, float driveKd, float driveStarti, float headingKp, float headingKi, float headingKd, float headingStarti);

  void controlArcade();
  void controlTank();
  void controlHolonomic();
};