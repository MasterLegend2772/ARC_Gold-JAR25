#include "vex.h"

/**
 * Setter method for tracker center distances.
 * The forward tracker center distance is the horizontal distance from the 
 * center of the robot to the center of the wheel the sensor is measuring.
 * The sideways tracker center distance is the vertical distance from the 
 * center of the robot to the center of the sideways wheel being measured.
 * If there's really no sideways wheel we set the center distance to 0 and
 * pretend the wheel never spins, which is equivalent to a no-drift robot.
 * 
 * @param ForwardTrackerCenterDistance A horizontal distance to the wheel center in inches.
 * @param SidewaysTrackerCenterDistance A vertical distance to the wheel center in inches.
 */

void Odom::setPhysicalDistances(float ForwardTrackerCenterDistance, float SidewaysTrackerCenterDistance){
  this -> ForwardTrackerCenterDistance = ForwardTrackerCenterDistance;
  this -> SidewaysTrackerCenterDistance = SidewaysTrackerCenterDistance;
}

/**
 * Resets the position, including tracking wheels.
 * Position is field-centric, and orientation is such that 0 degrees
 * is in the positive Y direction. Orientation can be provided with 
 * some flexibility, including less than 0 and greater than 360.
 * 
 * @param xPosition Field-centric x position of the robot.
 * @param yPosition Field-centric y position of the robot.
 * @param orientationDeg Field-centered, clockwise-positive, orientation.
 * @param ForwardTrackerPosition Current position of the sensor in inches.
 * @param SidewaysTrackerPosition Current position of the sensor in inches.
 */

void Odom::setPosition(float xPosition, float yPosition, float orientationDeg, float ForwardTrackerPosition, float SidewaysTrackerPosition){
  this -> ForwardTrackerPosition = ForwardTrackerPosition;
  this -> SidewaysTrackerPosition = SidewaysTrackerPosition;
  this -> xPosition = xPosition;
  this -> yPosition = yPosition;
  this -> orientationDeg = orientationDeg;
}

/**
 * Does the odometry math to update position
 * Uses the Pilons arc method outline here: https://wiki.purduesigbots.com/software/odometry
 * All the deltas are done by getting member variables and comparing them to 
 * the input. Ultimately this all works to update the public member variable
 * xPosition. This function needs to be run at 200Hz or so for best results.
 * 
 * @param ForwardTrackerPosition Current position of the sensor in inches.
 * @param SidewaysTrackerPosition Current position of the sensor in inches.
 * @param orientationDeg Field-centered, clockwise-positive, orientation.
 */

void Odom::updatePosition(float ForwardTrackerPosition, float SidewaysTrackerPosition, float orientationDeg){
  // this ->  always refers to the old version of the variable, so subtracting this -> x from x gives delta x.
  float Forward_delta = ForwardTrackerPosition - this -> ForwardTrackerPosition;
  float Sideways_delta = SidewaysTrackerPosition - this -> SidewaysTrackerPosition;
  this -> ForwardTrackerPosition = ForwardTrackerPosition;
  this -> SidewaysTrackerPosition = SidewaysTrackerPosition;
  float orientation_rad = to_rad(orientationDeg);
  float prev_orientation_rad = to_rad(this -> orientationDeg);
  float orientation_delta_rad = orientation_rad-prev_orientation_rad;
  this -> orientationDeg = orientationDeg;

  float local_xPosition;
  float local_yPosition;

  if (orientation_delta_rad == 0) {
    local_xPosition = Sideways_delta;
    local_yPosition = Forward_delta;
  } else {
    local_xPosition = (2 * sin(orientation_delta_rad / 2)) * ((Sideways_delta / orientation_delta_rad) + SidewaysTrackerCenterDistance); 
    local_yPosition = (2 * sin(orientation_delta_rad / 2)) * ((Forward_delta / orientation_delta_rad) + ForwardTrackerCenterDistance);
  }

  float local_polar_angle;
  float local_polar_length;

  if (local_xPosition == 0 && local_yPosition == 0){
    local_polar_angle = 0;
    local_polar_length = 0;
  } else {
    local_polar_angle = atan2(local_yPosition, local_xPosition); 
    local_polar_length = sqrt(pow(local_xPosition, 2) + pow(local_yPosition, 2)); 
  }

  float global_polar_angle = local_polar_angle - prev_orientation_rad - (orientation_delta_rad / 2);

  float xPosition_delta = local_polar_length * cos(global_polar_angle); 
  float yPosition_delta = local_polar_length * sin(global_polar_angle);

  xPosition += xPosition_delta;
  yPosition += yPosition_delta;
}