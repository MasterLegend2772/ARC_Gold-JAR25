/**
 * General-use odometry class with xPosition, yPosition, and
 * orientationDeg being the relevant outputs. This works for one
 * and two-tracker systems, and needs a gyro to get input angle.
 */

class Odom
{
private:
  float ForwardTrackerCenterDistance;
  float SidewaysTrackerCenterDistance;
  float ForwardTrackerPosition;
  float SidewaysTrackerPosition;
public:
  float xPosition;
  float yPosition;
  float orientationDeg;
  void setPosition(float xPosition, float yPosition, float orientationDeg, float ForwardTrackerPosition, float SidewaysTrackerPosition);
  void updatePosition(float ForwardTrackerPosition, float SidewaysTrackerPosition, float orientationDeg);
  void setPhysicalDistances(float ForwardTrackerCenterDistance, float SidewaysTrackerCenterDistance);
};