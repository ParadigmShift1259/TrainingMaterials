// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/NetworkButton.h>

RobotContainer::RobotContainer() 
    : m_autonomousCommand(&m_subsystem)
    , m_calculation()
{
  // Initialize all of your commands and subsystems here


  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kA).WhenPressed(
    frc2::InstantCommand(
      [this] {
        m_calculation.GetInitRPMS();
        // frc::SmartDashboard::PutNumber("SecondaryVel", vel.to<double>());
      },
      {}
    )
  );

}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
