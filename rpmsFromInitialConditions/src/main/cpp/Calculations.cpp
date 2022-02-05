#include "Calculations.h"
#include "Constants.h"
#include <units/math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/energy.h>

using namespace units;

Calculations::Calculations() {
  frc::SmartDashboard::PutNumber("HeightAboveHub", m_heightAboveHub.to<double>());
  frc::SmartDashboard::PutNumber("TargetHeight", m_heightTarget.to<double>());
  frc::SmartDashboard::PutNumber("RobotHeight", m_heightRobot.to<double>());
  frc::SmartDashboard::PutNumber("FloorHubDistance", m_xInput.to<double>());
  frc::SmartDashboard::PutNumber("TargetXDistance", m_xTarget.to<double>());
}

auto Calculations::HubHeightToMaxHeight() {
  auto aValue = (m_xInput * (m_heightTarget - m_heightRobot) - (m_xInput + m_xTarget) * (m_heightAboveHub - m_heightRobot)) / (m_xTarget * m_xInput * (m_xInput + m_xTarget));
  auto bValue = ((m_xInput + m_xTarget) * (m_xInput + m_xTarget) * (m_heightAboveHub - m_heightRobot) - m_xInput * m_xInput * (m_heightTarget - m_heightRobot)) / (m_xTarget * m_xInput * (m_xInput + m_xTarget));

  m_heightMax = -1.0 * bValue * bValue / (4.0 * aValue) + m_heightRobot;

  frc::SmartDashboard::PutNumber("HubHeightToMaxHeight", units::foot_t(m_heightMax).to<double>());

  return m_heightMax;
}

auto Calculations::GetTimeOne() {
  m_timeOne = math::sqrt(2.0 * (m_heightMax - m_heightRobot) / gravity);

  return m_timeOne;
}

auto Calculations::GetTimeTwo() {

  m_timeTwo = math::sqrt(2.0 * (m_heightMax - m_heightTarget) / gravity);

  return m_timeTwo;
}

auto Calculations::GetTotalTime() {
  m_timeTotal = GetTimeOne() + GetTimeTwo();

  return m_timeTotal;
}

auto Calculations::GetInitXVel() {
  m_velXInit = (m_xInput + m_xTarget) / GetTotalTime();

  return m_velXInit;
}

auto Calculations::GetInitYVel() {
  m_velYInit = math::sqrt(2.0 * gravity * (m_heightMax - m_heightRobot));

  return m_velYInit;
}

auto Calculations::GetInitVelWithAngle() {
  m_heightAboveHub = foot_t(frc::SmartDashboard::GetNumber("HeightAboveHub", 0.0));
  m_heightTarget = foot_t(frc::SmartDashboard::GetNumber("TargetHeight", 0.0));
  m_heightRobot = foot_t(frc::SmartDashboard::GetNumber("RobotHeight", 0.0));
  m_xInput = foot_t(frc::SmartDashboard::GetNumber("FloorHubDistance", 0.0));
  m_xTarget = foot_t(frc::SmartDashboard::GetNumber("TargetXDistance", 0.0));

  m_heightMax = HubHeightToMaxHeight();

  GetInitXVel();
  GetInitYVel();
  
  m_velInit = math::hypot(m_velXInit, m_velYInit);
  m_angleInit = math::atan(m_velYInit / m_velXInit);

// printf("InitVel %.3f\n", m_velInit.to<double>());
// printf("InitAngle %.3f\n", m_angleInit.to<double>());
  frc::SmartDashboard::PutNumber("InitVel", units::feet_per_second_t(m_velInit).to<double>());
  frc::SmartDashboard::PutNumber("InitAngle", m_angleInit.to<double>());

  return m_velInit;
}

revolutions_per_minute_t Calculations::GetInitRPMS() {
  GetInitVelWithAngle();

  auto aValue = flywheelRotInertia * (linearRegSlope - 1.0) * (linearRegSlope + linearRegSlope * rotInertiaRatio - rotInertiaRatio + 1);
  auto bValue = 2 * flywheelRotInertia * linearRegConst * (linearRegSlope + linearRegSlope * rotInertiaRatio - rotInertiaRatio);
  auto cValue = flywheelRotInertia * linearRegConst / radian_t (1.0) * linearRegConst / radian_t (1.0) * (rotInertiaRatio + 1.0) + cargoMass * m_velInit * m_velInit;
  

  m_rotVelInit = QuadraticFormula(aValue.to<double>(), bValue.to<double>(), cValue.to<double>(), (bool)true);
  m_rpmInit = m_rotVelInit;

  frc::SmartDashboard::PutNumber("InitRPM", m_rpmInit.to<double>());

  return m_rotVelInit;
}

radians_per_second_t Calculations::QuadraticFormula(double a, double b, double c, bool subtract) {
  auto outPut = radians_per_second_t(0.0);
  
  if (subtract == false)
    outPut = radians_per_second_t((-1.0 * b + sqrt(b * b - 4 * a * c)) / (2 * a));

  else
    outPut = radians_per_second_t((-1.0 * b - sqrt(b * b - 4 * a * c)) / (2 * a));

  return outPut;
}