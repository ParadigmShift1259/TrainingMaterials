// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/NetworkButton.h>

RobotContainer::RobotContainer() : m_flywheelSubsystem()
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
            double setpoint = m_calculation.GetInitRPMS().to<double>() / FlywheelConstants::kGearRatio;
            frc::SmartDashboard::PutNumber("InitSetPoint", setpoint);
            m_flywheelSubsystem.SetRPM(setpoint);
      },
      {}
    )
  );

  frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kB).WhenPressed(
    frc2::InstantCommand(
      [this] {
        m_calculation.CalculateAll();
      },
      {}
    )
  );


  frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kX).WhenPressed(
      frc2::InstantCommand(    
          [this] { 
              m_flywheelSubsystem.SetRPM(m_flywheelSubsystem.GetRPM() + 10.0);
            },
          {&m_flywheelSubsystem}
      )
  );

  frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kY).WhenPressed(
      frc2::InstantCommand(    
          [this] { 
              m_flywheelSubsystem.SetRPM(m_flywheelSubsystem.GetRPM() - 10.0);
            },
          {&m_flywheelSubsystem}
      )
  );
}
