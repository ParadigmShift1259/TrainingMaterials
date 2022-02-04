#include <units/time.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

using namespace units;

class Calculations {
 public:
  Calculations();
  auto HubHeightToMaxHeight();
  auto GetTimeOne();
  auto GetTimeTwo();
  auto GetTotalTime();
  auto GetInitXVel();
  auto GetInitYVel();
  auto GetInitVelWithAngle();
  revolutions_per_minute_t GetInitRPMS();
  radians_per_second_t QuadraticFormula(double a, double b, double c, bool subtract);

 private:
  
  second_t m_timeOne = second_t(0.0);
  second_t m_timeTwo = second_t(0.0);
  second_t m_timeTotal = second_t(0.0);

  meter_t m_heightAboveHub = meter_t(0.0);
  meter_t m_heightRobot = meter_t(0.0);
  meter_t m_heightTarget = meter_t(0.0);
  meter_t m_heightMax = meter_t(0.0);

  meter_t m_xInput = meter_t(0.0);
  meter_t m_xTarget = meter_t(0.0);

  meters_per_second_t m_velXInit = meters_per_second_t (0.0);
  meters_per_second_t m_velYInit = meters_per_second_t(0.0);
  meters_per_second_t m_velInit = meters_per_second_t(0.0);

  degree_t m_angleInit = degree_t(0.0);

  radians_per_second_t m_rotVelInit = radians_per_second_t(0.0);
  revolutions_per_minute_t m_rpmInit = revolutions_per_minute_t(0.0);
};