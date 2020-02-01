/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/i2c.h>
#include <rev/Rev2mDistanceSensor.h>
#include <rev/ColorSensorV3.h>


using namespace frc;
using namespace rev;


class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  void MuxSelect(uint8_t i);
  void ReadDistance(Rev2mDistanceSensor& distSensor, int sensorNum); 

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  I2C m_mux;

  Rev2mDistanceSensor* m_distsensor1;
  Rev2mDistanceSensor* m_distsensor2;
  ColorSensorV3* m_colorsensor;
};
