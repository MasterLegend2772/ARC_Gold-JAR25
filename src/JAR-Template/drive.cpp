#include "vex.h"

/**
 * Drive constructor for the chassis.
 * Even though there's only one constructor, there can be
 * huge differences in implementation depending on the drive style
 * selected.
 * 
 * @param driveSetup The style of drive, such as TANK_TWO_ROTATION.
 * @param DriveL Left motor group.
 * @param DriveR Right motor group.
 * @param gyro_port IMU port.
 * @param wheelDiameter Wheel diameter in inches.
 * @param wheelRatio External drive gear ratio.
 * @param gyroScale Scale factor in degrees.
 * @param DriveLF_port Left front port for holonomic drives.
 * @param DriveRF_port Right front port for holonomic drives.
 * @param DriveLB_port Left back port for holonomic drives.
 * @param DriveRB_port Right back port for holonomic drives.
 * @param ForwardTracker_port Port for the forward tracker.
 * @param ForwardTrackerDiameter Diameter in inches.
 * @param ForwardTrackerCenterDistance Horizontal distance in inches.
 * @param SidewaysTracker_port Port for the sideways tracker.
 * @param SidewaysTrackerDiameter Diameter in inches.
 * @param SidewaysTrackerCenterDistance Vertical distance in inches.
 */

Drive::Drive(enum::driveSetup driveSetup, motor_group DriveL, motor_group DriveR, 
int gyro_port, float wheelDiameter, float wheelRatio, float gyroScale, 
int DriveLF_port, int DriveRF_port, int DriveLB_port, int DriveRB_port, 
int ForwardTracker_port, float ForwardTrackerDiameter, float ForwardTrackerCenterDistance, 
int SidewaysTracker_port, float SidewaysTrackerDiameter, float SidewaysTrackerCenterDistance) :
  wheelDiameter(wheelDiameter),
  wheelRatio(wheelRatio),
  gyroScale(gyroScale),
  driveInToDegRatio(wheelRatio/360.0*M_PI*wheelDiameter),
  ForwardTrackerCenterDistance(ForwardTrackerCenterDistance),
  ForwardTrackerDiameter(ForwardTrackerDiameter),
  ForwardTrackerInToDegRatio(M_PI*ForwardTrackerDiameter/360.0),
  SidewaysTrackerCenterDistance(SidewaysTrackerCenterDistance),
  SidewaysTrackerDiameter(SidewaysTrackerDiameter),
  SidewaysTrackerInToDegRatio(M_PI*SidewaysTrackerDiameter/360.0),
  driveSetup(driveSetup),
  DriveL(DriveL),
  DriveR(DriveR),
  Gyro(inertial(gyro_port)),
  DriveLF(abs(DriveLF_port), is_reversed(DriveLF_port)),
  DriveRF(abs(DriveRF_port), is_reversed(DriveRF_port)),
  DriveLB(abs(DriveLB_port), is_reversed(DriveLB_port)),
  DriveRB(abs(DriveRB_port), is_reversed(DriveRB_port)),
  RForwardTracker(ForwardTracker_port),
  RSidewaysTracker(SidewaysTracker_port),
  EForwardTracker(ThreeWire.Port[to_port(ForwardTracker_port)]),
  ESidewaysTracker(ThreeWire.Port[to_port(SidewaysTracker_port)])
{
    if (driveSetup == TANK_ONE_FORWARD_ENCODER || driveSetup == TANK_ONE_FORWARD_ROTATION || driveSetup == ZERO_TRACKER_ODOM) {
      odom.setPhysicalDistances(ForwardTrackerCenterDistance, 0);
    } 
    if (driveSetup == TANK_ONE_SIDEWAYS_ENCODER || driveSetup == TANK_ONE_SIDEWAYS_ROTATION || 
    driveSetup == TANK_TWO_ENCODER || driveSetup == TANK_TWO_ROTATION ||
    driveSetup == HOLONOMIC_TWO_ENCODER || driveSetup == HOLONOMIC_TWO_ROTATION) {
      odom.setPhysicalDistances(ForwardTrackerCenterDistance, SidewaysTrackerCenterDistance);
    }
}

/**
 * Drives each side of the chassis at the specified voltage.
 * 
 * @param leftVoltage Voltage out of 12.
 * @param rightVoltage Voltage out of 12.
 */

void Drive::driveWithVoltage(float leftVoltage, float rightVoltage) {
  DriveL.spin(fwd, leftVoltage, volt);
  DriveR.spin(fwd, rightVoltage,volt);
}

/**
 * Resets default turn constants.
 * Turning includes turnToAngle() and turnToPoint().
 * 
 * @param turnMaxVoltage Max voltage out of 12.
 * @param turnKp Proportional constant.
 * @param turnKi Integral constant.
 * @param turnKd Derivative constant.
 * @param turnStarti Minimum angle in degrees for integral to begin.
 */

void Drive::setTurnConstants(float turnMaxVoltage, float turnKp, float turnKi, float turnKd, float turnStarti) {
  this -> turnMaxVoltage = turnMaxVoltage;
  this -> turnKp = turnKp;
  this -> turnKi = turnKi;
  this -> turnKd = turnKd;
  this -> turnStarti = turnStarti;
} 

/**
 * Resets default drive constants.
 * Driving includes driveDistance(), driveToPoint(), and
 * holonomic_driveToPoint().
 * 
 * @param driveMaxVoltage Max voltage out of 12.
 * @param driveKp Proportional constant.
 * @param driveKi Integral constant.
 * @param driveKd Derivative constant.
 * @param driveStarti Minimum distance in inches for integral to begin
 */

void Drive::setDriveTrainConstants(float driveMaxVoltage, float driveKp, float driveKi, float driveKd, float driveStarti) {
  this -> driveMaxVoltage = driveMaxVoltage;
  this -> driveKp = driveKp;
  this -> driveKi = driveKi;
  this -> driveKd = driveKd;
  this -> driveStarti = driveStarti;
} 

/**
 * Resets default heading constants.
 * Heading control keeps the robot facing the right direction
 * and is part of driveDistance(), driveToPoint(), and
 * holonomic_driveToPoint.
 * 
 * @param headingMaxVoltage Max voltage out of 12.
 * @param headingKp Proportional constant.
 * @param headingKi Integral constant.
 * @param headingKd Derivative constant.
 * @param headingStarti Minimum angle in degrees for integral to begin.
 */

void Drive::setHeadingConstants(float headingMaxVoltage, float headingKp, float headingKi, float headingKd, float headingStarti) {
  this -> headingMaxVoltage = headingMaxVoltage;
  this -> headingKp = headingKp;
  this -> headingKi = headingKi;
  this -> headingKd = headingKd;
  this -> headingStarti = headingStarti;
}

/**
 * Resets default swing constants.
 * Swing control holds one side of the drive still and turns with the other.
 * Only leftSwingToAngle() and rightSwingToAngle() use these constants.
 * 
 * @param swingMaxVoltage Max voltage out of 12.
 * @param swingKp Proportional constant.
 * @param swingKi Integral constant.
 * @param swingKd Derivative constant.
 * @param swingStarti Minimum angle in degrees for integral to begin.
 */

void Drive::setSwingConstants(float swingMaxVoltage, float swingKp, float swingKi, float swingKd, float swingStarti) {
  this -> swingMaxVoltage = swingMaxVoltage;
  this -> swingKp = swingKp;
  this -> swingKi = swingKi;
  this -> swingKd = swingKd;
  this -> swingStarti = swingStarti;
} 

/**
 * Resets default turn exit conditions.
 * The robot exits when error is less than settle_error for a duration of settle_time, 
 * or if the function has gone on for longer than timeout.
 * 
 * @param turnSettleError Error to be considered settled in degrees.
 * @param turnSettleTime Time to be considered settled in milliseconds.
 * @param turnTimeout Time before quitting and move on in milliseconds.
 */

void Drive::setTurnExitConditions(float turnSettleError, float turnSettleTime, float turnTimeout) {
  this -> turnSettleError = turnSettleError;
  this -> turnSettleTime = turnSettleTime;
  this -> turnTimeout = turnTimeout;
}

/**
 * Resets default drive exit conditions.
 * The robot exits when error is less than settle_error for a duration of settle_time, 
 * or if the function has gone on for longer than timeout.
 * 
 * @param driveSettleError Error to be considered settled in inches.
 * @param driveSettleTime Time to be considered settled in milliseconds.
 * @param driveTimeout Time before quitting and move on in milliseconds.
 */

void Drive::setDriveExitConditions(float driveSettleError, float driveSettleTime, float driveTimeout) {
  this -> driveSettleError = driveSettleError;
  this -> driveSettleTime = driveSettleTime;
  this -> driveTimeout = driveTimeout;
}

/**
 * Resets default swing exit conditions.
 * The robot exits when error is less than settle_error for a duration of settle_time, 
 * or if the function has gone on for longer than timeout.
 * 
 * @param swingSettleError Error to be considered settled in degrees.
 * @param swingSettleTime Time to be considered settled in milliseconds.
 * @param swingTimeout Time before quitting and move on in milliseconds.
 */

void Drive::setSwingExitConditions(float swingSettleError, float swingSettleTime, float swingTimeout) {
  this -> swingSettleError = swingSettleError;
  this -> swingSettleTime = swingSettleTime;
  this -> swingTimeout = swingTimeout;
}

/**
 * Gives the drive's absolute heading with Gyro correction.
 * 
 * @return Gyro scale-corrected heading in the range [0, 360).
 */

float Drive::getAbsoluteHeading() { 
  return (reduce_0_to_360(Gyro.rotation() * 360.0 / gyroScale)); 
}

/**
 * Gets the motor group's position and converts to inches.
 * 
 * @return Left position in inches.
 */

float Drive::getLeftPositionIn() {
  return (DriveL.position(deg) * driveInToDegRatio);
}

/**
 * Gets the motor group's position and converts to inches.
 * 
 * @return Right position in inches.
 */

float Drive::getRightPositionIn() {
  return (DriveR.position(deg) * driveInToDegRatio);
}

/**
 * Stops both sides of the drive with the desired mode.
 * 
 * @param mode hold, brake, or stop
 */

void Drive::drive_stop(vex::brakeType mode) {
  DriveL.stop(mode);
  DriveR.stop(mode);
}

/**
 * Turns the robot to a field-centric angle.
 * Optimizes direction, so it turns whichever way is closer to the 
 * current heading of the robot.
 * 
 * @param angle Desired angle in degrees.
 */

void Drive::turnToAngle(float angle) {
  turnToAngle(angle, turnMaxVoltage, turnSettleError, turnSettleTime, turnTimeout, turnKp, turnKi, turnKd, turnStarti);
}

void Drive::turnToAngle(float angle, float turnMaxVoltage) {
  turnToAngle(angle, turnMaxVoltage, turnSettleError, turnSettleTime, turnTimeout, turnKp, turnKi, turnKd, turnStarti);
}

void Drive::turnToAngle(float angle, float turnMaxVoltage, float turnSettleError, float turnSettleTime, float turnTimeout) {
  turnToAngle(angle, turnMaxVoltage, turnSettleError, turnSettleTime, turnTimeout, turnKp, turnKi, turnKd, turnStarti);
}

void Drive::turnToAngle(float angle, float turnMaxVoltage, float turnSettleError, float turnSettleTime, float turnTimeout, float turnKp, float turnKi, float turnKd, float turnStarti) {
  PID turnPID(reduce_negative_180_to_180(angle - getAbsoluteHeading()), turnKp, turnKi, turnKd, turnStarti, turnSettleError, turnSettleTime, turnTimeout);
  while ( !turnPID.isSettled() ) {
    float error = reduce_negative_180_to_180(angle - getAbsoluteHeading());
    float output = turnPID.compute(error);
    output = clamp(output, -turnMaxVoltage, turnMaxVoltage);
    driveWithVoltage(output, -output);
    task::sleep(10);
  }
}

/**
 * Drives the robot a given distance with a given heading.
 * Drive distance does not optimize for direction, so it won't try
 * to drive at the opposite heading from the one given to get there faster.
 * You can control the heading, but if you choose not to, it will drive with the
 * heading it's currently facing. It uses the average of the left and right
 * motor groups to calculate distance driven.
 * 
 * @param distance Desired distance in inches.
 * @param heading Desired heading in degrees.
 */

void Drive::driveDistance(float distance) {
  driveDistance(distance, getAbsoluteHeading(), driveMaxVoltage, headingMaxVoltage, driveSettleError, driveSettleTime, driveTimeout, driveKp, driveKi, driveKd, driveStarti, headingKp, headingKi, headingKd, headingStarti);
}

void Drive::driveDistance(float distance, float heading) {
  driveDistance(distance, heading, driveMaxVoltage, headingMaxVoltage, driveSettleError, driveSettleTime, driveTimeout, driveKp, driveKi, driveKd, driveStarti, headingKp, headingKi, headingKd, headingStarti);
}

void Drive::driveDistance(float distance, float heading, float driveMaxVoltage, float headingMaxVoltage) {
  driveDistance(distance, heading, driveMaxVoltage, headingMaxVoltage, driveSettleError, driveSettleTime, driveTimeout, driveKp, driveKi, driveKd, driveStarti, headingKp, headingKi, headingKd, headingStarti);
}

void Drive::driveDistance(float distance, float heading, float driveMaxVoltage, float headingMaxVoltage, float driveSettleError, float driveSettleTime, float driveTimeout) {
  driveDistance(distance, heading, driveMaxVoltage, headingMaxVoltage, driveSettleError, driveSettleTime, driveTimeout, driveKp, driveKi, driveKd, driveStarti, headingKp, headingKi, headingKd, headingStarti);
}

void Drive::driveDistance(float distance, float heading, float driveMaxVoltage, float headingMaxVoltage, float driveSettleError, float driveSettleTime, float driveTimeout, float driveKp, float driveKi, float driveKd, float driveStarti, float headingKp, float headingKi, float headingKd, float headingStarti) {
  PID drivePID(distance, driveKp, driveKi, driveKd, driveStarti, driveSettleError, driveSettleTime, driveTimeout);
  PID headingPID(reduce_negative_180_to_180(heading - getAbsoluteHeading()), headingKp, headingKi, headingKd, headingStarti);
  float start_average_position = (getLeftPositionIn()+getRightPositionIn())/2.0;
  float average_position = start_average_position;
  while (drivePID.isSettled() == false) {
    average_position = (getLeftPositionIn()+getRightPositionIn())/2.0;
    float drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - getAbsoluteHeading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -driveMaxVoltage, driveMaxVoltage);
    heading_output = clamp(heading_output, -headingMaxVoltage, headingMaxVoltage);

    driveWithVoltage(drive_output+heading_output, drive_output-heading_output);
    task::sleep(10);
  }
}

/**
 * Turns to a given angle with only one side of the drivetrain.
 * Like turnToAngle(), is optimized for turning the shorter
 * direction.
 * 
 * @param angle Desired angle in degrees.
 */

void Drive::leftSwingToAngle(float angle) {
  leftSwingToAngle(angle, swingMaxVoltage, swingSettleError, swingSettleTime, swingTimeout, swingKp, swingKi, swingKd, swingStarti);
}

void Drive::leftSwingToAngle(float angle, float swingMaxVoltage, float swingSettleError, float swingSettleTime, float swingTimeout, float swingKp, float swingKi, float swingKd, float swingStarti) {
  PID swingPID(reduce_negative_180_to_180(angle - getAbsoluteHeading()), swingKp, swingKi, swingKd, swingStarti, swingSettleError, swingSettleTime, swingTimeout);
  while (swingPID.isSettled() == false) {
    float error = reduce_negative_180_to_180(angle - getAbsoluteHeading());
    float output = swingPID.compute(error);
    output = clamp(output, -turnMaxVoltage, turnMaxVoltage);
    DriveL.spin(fwd, output, volt);
    DriveR.stop(hold);
    task::sleep(10);
  }
}

void Drive::rightSwingToAngle(float angle) {
  rightSwingToAngle(angle, swingMaxVoltage, swingSettleError, swingSettleTime, swingTimeout, swingKp, swingKi, swingKd, swingStarti);
}

void Drive::rightSwingToAngle(float angle, float swingMaxVoltage, float swingSettleError, float swingSettleTime, float swingTimeout, float swingKp, float swingKi, float swingKd, float swingStarti) {
  PID swingPID(reduce_negative_180_to_180(angle - getAbsoluteHeading()), swingKp, swingKi, swingKd, swingStarti, swingSettleError, swingSettleTime, swingTimeout);
  while (swingPID.isSettled() == false) {
    float error = reduce_negative_180_to_180(angle - getAbsoluteHeading());
    float output = swingPID.compute(error);
    output = clamp(output, -turnMaxVoltage, turnMaxVoltage);
    DriveR.spin(reverse, output, volt);
    DriveL.stop(hold);
    task::sleep(10);
  }
}

/**
 * Depending on the drive style, gets the tracker's position.
 * 
 * @return The tracker position.
 */

float Drive::getForwardTrackerPosition() {
  if (driveSetup==ZERO_TRACKER_ODOM || driveSetup == TANK_ONE_SIDEWAYS_ENCODER || driveSetup == TANK_ONE_SIDEWAYS_ROTATION) {
    return (getRightPositionIn());
  }
  if (driveSetup==TANK_ONE_FORWARD_ENCODER || driveSetup == TANK_TWO_ENCODER || driveSetup == HOLONOMIC_TWO_ENCODER) {
    return (EForwardTracker.position(deg) * ForwardTrackerInToDegRatio);
  } else {
    return (RForwardTracker.position(deg) * ForwardTrackerInToDegRatio);
  }
}

/**
 * Depending on the drive style, gets the tracker's position.
 * 
 * @return The tracker position.
 */

float Drive::getSidewaysTrackerPosition() {
  if (driveSetup==TANK_ONE_FORWARD_ENCODER || driveSetup == TANK_ONE_FORWARD_ROTATION || driveSetup == ZERO_TRACKER_ODOM) {
    return (0);
  } else if (driveSetup == TANK_TWO_ENCODER || driveSetup == HOLONOMIC_TWO_ENCODER || driveSetup == TANK_ONE_SIDEWAYS_ENCODER) {
    return (ESidewaysTracker.position(deg)*SidewaysTrackerInToDegRatio);
  } else {
    return (RSidewaysTracker.position(deg)*SidewaysTrackerInToDegRatio);
  }
}

/**
 * Background task for updating the odometry.
 */

void Drive::positionTrack() {
  while (1) {
    odom.updatePosition(getForwardTrackerPosition(), getSidewaysTrackerPosition(), getAbsoluteHeading());
    task::sleep(5);
  }
}

/**
 * Resets the robot's heading.
 * For example, at the beginning of auton, if your robot starts at
 * 45 degrees, so setHeading(45) and the robot will know which way 
 * it's facing.
 * 
 * @param orientationDeg Desired heading in degrees.
 */

void Drive::setHeading(float orientationDeg) {
  Gyro.setRotation(orientationDeg * gyroScale/360.0, deg);
}

/**
 * Resets the robot's coordinates and heading.
 * This is for odom-using robots to specify where the bot is at the beginning
 * of the match.
 * 
 * @param xPosition Robot's x in inches.
 * @param yPosition Robot's y in inches.
 * @param orientationDeg Desired heading in degrees.
 */

void Drive::setCoordinates(float xPosition, float yPosition, float orientationDeg) {
  odom.setPosition(xPosition, yPosition, orientationDeg, getForwardTrackerPosition(), getSidewaysTrackerPosition());
  setHeading(orientationDeg);
  odomTask = task(positionTrackTask);
}

/**
 * Gets the robot's x.
 * 
 * @return The robot's x position in inches.
 */

float Drive::getXPosition() {
  return (odom.xPosition);
}

/**
 * Gets the robot's y.
 * 
 * @return The robot's y position in inches.
 */

float Drive::getYPosition() {
  return (odom.yPosition);
}

/**
 * Drives to a specified point on the field.
 * Uses the double-PID method, with one for driving and one for heading correction.
 * The drive error is the euclidean distance to the desired point, and the heading error
 * is the turn correction from the current heading to the desired point. Uses optimizations
 * like driving backwards whenever possible and scaling the drive output with the cosine
 * of the angle to the point.
 * 
 * @param xPosition Desired x position in inches.
 * @param yPosition Desired y position in inches.
 */

void Drive::driveToPoint(float xPosition, float yPosition) {
  driveToPoint(xPosition, yPosition, driveMinVoltage, driveMaxVoltage, headingMaxVoltage, driveSettleError, driveSettleTime, driveTimeout, driveKp, driveKi, driveKd, driveStarti, headingKp, headingKi, headingKd, headingStarti);
}

void Drive::driveToPoint(float xPosition, float yPosition, float driveMinVoltage, float driveMaxVoltage, float headingMaxVoltage) {
  driveToPoint(xPosition, yPosition, driveMinVoltage, driveMaxVoltage, headingMaxVoltage, driveSettleError, driveSettleTime, driveTimeout, driveKp, driveKi, driveKd, driveStarti, headingKp, headingKi, headingKd, headingStarti);
}

void Drive::driveToPoint(float xPosition, float yPosition, float driveMinVoltage, float driveMaxVoltage, float headingMaxVoltage, float driveSettleError, float driveSettleTime, float driveTimeout) {
  driveToPoint(xPosition, yPosition, driveMinVoltage, driveMaxVoltage, headingMaxVoltage, driveSettleError, driveSettleTime, driveTimeout, driveKp, driveKi, driveKd, driveStarti, headingKp, headingKi, headingKd, headingStarti);
}

void Drive::driveToPoint(float xPosition, float yPosition, float driveMinVoltage, float driveMaxVoltage, float headingMaxVoltage, float driveSettleError, float driveSettleTime, float driveTimeout, float driveKp, float driveKi, float driveKd, float driveStarti, float headingKp, float headingKi, float headingKd, float headingStarti) {
  PID drivePID(hypot(xPosition-getXPosition(),yPosition-getYPosition()), driveKp, driveKi, driveKd, driveStarti, driveSettleError, driveSettleTime, driveTimeout);
  float start_angle_deg = to_deg(atan2(xPosition-getXPosition(),yPosition-getYPosition()));
  PID headingPID(start_angle_deg-getAbsoluteHeading(), headingKp, headingKi, headingKd, headingStarti);
  bool line_settled = false;
  bool prev_line_settled = is_line_settled(xPosition, yPosition, start_angle_deg, getXPosition(), getYPosition());
  while (!drivePID.isSettled()) {
    line_settled = is_line_settled(xPosition, yPosition, start_angle_deg, getXPosition(), getYPosition());
    if (line_settled && !prev_line_settled) { break; }
    prev_line_settled = line_settled;

    float drive_error = hypot(xPosition-getXPosition(),yPosition-getYPosition());
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(xPosition-getXPosition(),yPosition-getYPosition()))-getAbsoluteHeading());
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);
    
    if (drive_error<driveSettleError) { heading_output = 0; }

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*driveMaxVoltage, fabs(heading_scale_factor)*driveMaxVoltage);
    heading_output = clamp(heading_output, -headingMaxVoltage, headingMaxVoltage);

    drive_output = clamp_min_voltage(drive_output, driveMinVoltage);

    driveWithVoltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
    task::sleep(10);
  }
}

/**
 * Drives to a specified point and orientation on the field.
 * Uses a boomerang controller. The carrot point is back from the target
 * by the same distance as the robot's distance to the target, times the lead. The
 * robot always tries to go to the carrot, which is constantly moving, and the
 * robot eventually gets into position. The heading correction is optimized to only
 * try to reach the correct angle when drive error is low, and the robot will drive 
 * backwards to reach a pose if it's faster. .5 is a reasonable value for the lead. 
 * The setback parameter is used to glide into position more effectively. It is
 * the distance back from the target that the robot tries to drive to first.
 * 
 * @param xPosition Desired x position in inches.
 * @param yPosition Desired y position in inches.
 * @param angle Desired orientation in degrees.
 * @param lead Constant scale factor that determines how far away the carrot point is. 
 * @param setback Distance in inches from target by which the carrot is always pushed back.
 * @param driveMinVoltage Minimum voltage on the drive, used for chaining movements.
 */

void Drive::driveToPose(float xPosition, float yPosition, float angle) {
  driveToPose(xPosition, yPosition, angle, boomerangLead, boomerang_setback, driveMinVoltage, driveMaxVoltage, headingMaxVoltage, driveSettleError, driveSettleTime, driveTimeout, driveKp, driveKi, driveKd, driveStarti, headingKp, headingKi, headingKd, headingStarti);
}

void Drive::driveToPose(float xPosition, float yPosition, float angle, float lead, float setback, float driveMinVoltage) {
  driveToPose(xPosition, yPosition, angle, lead, setback, driveMinVoltage, driveMaxVoltage, headingMaxVoltage, driveSettleError, driveSettleTime, driveTimeout, driveKp, driveKi, driveKd, driveStarti, headingKp, headingKi, headingKd, headingStarti);
}

void Drive::driveToPose(float xPosition, float yPosition, float angle, float lead, float setback, float driveMinVoltage, float driveMaxVoltage, float headingMaxVoltage) {
  driveToPose(xPosition, yPosition, angle, lead, setback, driveMinVoltage, driveMaxVoltage, headingMaxVoltage, driveSettleError, driveSettleTime, driveTimeout, driveKp, driveKi, driveKd, driveStarti, headingKp, headingKi, headingKd, headingStarti);
}


void Drive::driveToPose(float xPosition, float yPosition, float angle, float lead, float setback, float driveMinVoltage, float driveMaxVoltage, float headingMaxVoltage, float driveSettleError, float driveSettleTime, float driveTimeout) {
  driveToPose(xPosition, yPosition, angle, lead, setback, driveMinVoltage, driveMaxVoltage, headingMaxVoltage, driveSettleError, driveSettleTime, driveTimeout, driveKp, driveKi, driveKd, driveStarti, headingKp, headingKi, headingKd, headingStarti);
}

void Drive::driveToPose(float xPosition, float yPosition, float angle, float lead, float setback, float driveMinVoltage, float driveMaxVoltage, float headingMaxVoltage, float driveSettleError, float driveSettleTime, float driveTimeout, float driveKp, float driveKi, float driveKd, float driveStarti, float headingKp, float headingKi, float headingKd, float headingStarti) {
  float target_distance = hypot(xPosition-getXPosition(),yPosition-getYPosition());
  PID drivePID(target_distance, driveKp, driveKi, driveKd, driveStarti, driveSettleError, driveSettleTime, driveTimeout);
  PID headingPID(to_deg(atan2(xPosition-getXPosition(),yPosition-getYPosition()))-getAbsoluteHeading(), headingKp, headingKi, headingKd, headingStarti);
  bool line_settled = is_line_settled(xPosition, yPosition, angle, getXPosition(), getYPosition());
  bool prev_line_settled = is_line_settled(xPosition, yPosition, angle, getXPosition(), getYPosition());
  bool crossed_center_line = false;
  bool center_line_side = is_line_settled(xPosition, yPosition, angle+90, getXPosition(), getYPosition());
  bool prev_center_line_side = center_line_side;
  while (!drivePID.isSettled()) {
    line_settled = is_line_settled(xPosition, yPosition, angle, getXPosition(), getYPosition());
    if (line_settled && !prev_line_settled) { break; }
    prev_line_settled = line_settled;

    center_line_side = is_line_settled(xPosition, yPosition, angle+90, getXPosition(), getYPosition());
    if (center_line_side != prev_center_line_side) {
      crossed_center_line = true;
    }

    target_distance = hypot(xPosition-getXPosition(),yPosition-getYPosition());

    float carrot_X = xPosition - sin(to_rad(angle)) * (lead * target_distance + setback);
    float carrot_Y = yPosition - cos(to_rad(angle)) * (lead * target_distance + setback);

    float drive_error = hypot(carrot_X-getXPosition(),carrot_Y-getYPosition());
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(carrot_X-getXPosition(),carrot_Y-getYPosition()))-getAbsoluteHeading());

    if (drive_error<driveSettleError || crossed_center_line || drive_error < setback) { 
      heading_error = reduce_negative_180_to_180(angle-getAbsoluteHeading()); 
      drive_error = target_distance;
    }
    
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*driveMaxVoltage, fabs(heading_scale_factor)*driveMaxVoltage);
    heading_output = clamp(heading_output, -headingMaxVoltage, headingMaxVoltage);

    drive_output = clamp_min_voltage(drive_output, driveMinVoltage);

    driveWithVoltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
    task::sleep(10);
  }
}

/**
 * Turns to a specified point on the field.
 * Functions similarly to turnToAngle() except with a point. The
 * extraAngleDeg parameter turns the robot extra relative to the 
 * desired target. For example, if you want the back of your robot
 * to point at (36, 42), you would run turnToPoint(36, 42, 180).
 * 
 * @param xPosition Desired x position in inches.
 * @param yPosition Desired y position in inches.
 * @param extraAngleDeg Angle turned past the desired heading in degrees.
 */

void Drive::turnToPoint(float xPosition, float yPosition) {
  turnToPoint(xPosition, yPosition, 0, turnMaxVoltage, turnSettleError, turnSettleTime, turnTimeout, turnKp, turnKi, turnKd, turnStarti);
}

void Drive::turnToPoint(float xPosition, float yPosition, float extraAngleDeg) {
  turnToPoint(xPosition, yPosition, extraAngleDeg, turnMaxVoltage, turnSettleError, turnSettleTime, turnTimeout, turnKp, turnKi, turnKd, turnStarti);
}

void Drive::turnToPoint(float xPosition, float yPosition, float extraAngleDeg, float turnMaxVoltage, float turnSettleError, float turnSettleTime, float turnTimeout) {
  turnToPoint(xPosition, yPosition, extraAngleDeg, turnMaxVoltage, turnSettleError, turnSettleTime, turnTimeout, turnKp, turnKi, turnKd, turnStarti);
}

void Drive::turnToPoint(float xPosition, float yPosition, float extraAngleDeg, float turnMaxVoltage, float turnSettleError, float turnSettleTime, float turnTimeout, float turnKp, float turnKi, float turnKd, float turnStarti) {
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(xPosition-getXPosition(),yPosition-getYPosition())) - getAbsoluteHeading()), turnKp, turnKi, turnKd, turnStarti, turnSettleError, turnSettleTime, turnTimeout);
  while (turnPID.isSettled() == false) {
    float error = reduce_negative_180_to_180(to_deg(atan2(xPosition-getXPosition(),yPosition-getYPosition())) - getAbsoluteHeading() + extraAngleDeg);
    float output = turnPID.compute(error);
    output = clamp(output, -turnMaxVoltage, turnMaxVoltage);
    driveWithVoltage(output, -output);
    task::sleep(10);
  }
}

/**
 * Drives and turns simultaneously to a desired pose.
 * Uses two PID loops, one drive and one heading to drive and turn
 * at the same time. Optimized to turn the quicker direction and only
 * exits once both PID loops have settled. It uses the heading constants
 * for heading but the turn exit conditions to settle.
 * 
 * @param xPosition Desired x position in inches.
 * @param yPosition Desired y position in inches.
 * @param angle Desired ending angle in degrees.
 */

void Drive::holonomicDriveToPose(float xPosition, float yPosition) {
  holonomicDriveToPose(xPosition, yPosition, getAbsoluteHeading(), driveMaxVoltage, headingMaxVoltage, driveSettleError, driveSettleTime, driveTimeout, driveKp, driveKi, driveKd, driveStarti, headingKp, headingKi, headingKd, headingStarti);
}

void Drive::holonomicDriveToPose(float xPosition, float yPosition, float angle) {
  holonomicDriveToPose(xPosition, yPosition, angle, driveMaxVoltage, headingMaxVoltage, driveSettleError, driveSettleTime, driveTimeout, driveKp, driveKi, driveKd, driveStarti, headingKp, headingKi, headingKd, headingStarti);
}

void Drive::holonomicDriveToPose(float xPosition, float yPosition, float angle, float driveMaxVoltage, float headingMaxVoltage) {
  holonomicDriveToPose(xPosition, yPosition, angle, driveMaxVoltage, headingMaxVoltage, driveSettleError, driveSettleTime, driveTimeout, driveKp, driveKi, driveKd, driveStarti, headingKp, headingKi, headingKd, headingStarti);
}

void Drive::holonomicDriveToPose(float xPosition, float yPosition, float angle, float driveMaxVoltage, float headingMaxVoltage, float driveSettleError, float driveSettleTime, float driveTimeout) {
  holonomicDriveToPose(xPosition, yPosition, angle, driveMaxVoltage, headingMaxVoltage, driveSettleError, driveSettleTime, driveTimeout, driveKp, driveKi, driveKd, driveStarti, headingKp, headingKi, headingKd, headingStarti);
}

void Drive::holonomicDriveToPose(float xPosition, float yPosition, float angle, float driveMaxVoltage, float headingMaxVoltage, float driveSettleError, float driveSettleTime, float driveTimeout, float driveKp, float driveKi, float driveKd, float driveStarti, float headingKp, float headingKi, float headingKd, float headingStarti) {
  PID drivePID(hypot(xPosition-getXPosition(),yPosition-getYPosition()), driveKp, driveKi, driveKd, driveStarti, driveSettleError, driveSettleTime, driveTimeout);
  PID turnPID(angle-getAbsoluteHeading(), headingKp, headingKi, headingKd, headingStarti, turnSettleError, turnSettleTime, turnTimeout);
  while ( !(drivePID.isSettled() && turnPID.isSettled()) ) {
    float drive_error = hypot(xPosition-getXPosition(),yPosition-getYPosition());
    float turn_error = reduce_negative_180_to_180(angle-getAbsoluteHeading());

    float drive_output = drivePID.compute(drive_error);
    float turn_output = turnPID.compute(turn_error);

    drive_output = clamp(drive_output, -driveMaxVoltage, driveMaxVoltage);
    turn_output = clamp(turn_output, -headingMaxVoltage, headingMaxVoltage);

    float heading_error = atan2(yPosition-getYPosition(), xPosition-getXPosition());

    DriveLF.spin(fwd, drive_output*cos(to_rad(getAbsoluteHeading()) + heading_error - M_PI/4) + turn_output, volt);
    DriveLB.spin(fwd, drive_output*cos(-to_rad(getAbsoluteHeading()) - heading_error + 3*M_PI/4) + turn_output, volt);
    DriveRB.spin(fwd, drive_output*cos(to_rad(getAbsoluteHeading()) + heading_error - M_PI/4) - turn_output, volt);
    DriveRF.spin(fwd, drive_output*cos(-to_rad(getAbsoluteHeading()) - heading_error + 3*M_PI/4) - turn_output, volt);
    task::sleep(10);
  }
}

/**
 * Controls a chassis with left stick throttle and right stick turning.
 * Default deadband is 5.
 */

void Drive::controlArcade() {
  float throttle = deadband(controller(primary).Axis3.value(), 5);
  float turn = deadband(controller(primary).Axis1.value(), 5);
  DriveL.spin(fwd, to_volt(throttle+turn), volt);
  DriveR.spin(fwd, to_volt(throttle-turn), volt);
}

/**
 * Controls a chassis with left stick throttle and strafe, and right stick turning.
 * Default deadband is 5.
 */

void Drive::controlHolonomic() {
  float throttle = deadband(controller(primary).Axis3.value(), 5);
  float turn = deadband(controller(primary).Axis1.value(), 5);
  float strafe = deadband(controller(primary).Axis4.value(), 5);
  DriveLF.spin(fwd, to_volt(throttle+turn+strafe), volt);
  DriveRF.spin(fwd, to_volt(throttle-turn-strafe), volt);
  DriveLB.spin(fwd, to_volt(throttle+turn-strafe), volt);
  DriveRB.spin(fwd, to_volt(throttle-turn+strafe), volt);
}

/**
 * Controls a chassis with left stick left drive and right stick right drive.
 * Default deadband is 5.
 */

void Drive::controlTank() {
  float leftthrottle = deadband(controller(primary).Axis3.value(), 5);
  float rightthrottle = deadband(controller(primary).Axis2.value(), 5);
  DriveL.spin(fwd, to_volt(leftthrottle), volt);
  DriveR.spin(fwd, to_volt(rightthrottle), volt);
}

/**
 * Tracking task to run in the background.
 */

int Drive::positionTrackTask() {
  chassis.positionTrack();
  return (0);
}