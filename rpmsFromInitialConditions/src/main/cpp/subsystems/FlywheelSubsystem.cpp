// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/voltage.h>

#include "subsystems/FlywheelSubsystem.h"

FlywheelSubsystem::FlywheelSubsystem() 
  : m_flywheelmotor(kMotorPort, CANSparkMax::MotorType::kBrushless){
    m_flywheelmotor.SetIdleMode(CANSparkMax::IdleMode::kCoast);
    m_flywheelmotor.SetClosedLoopRampRate(kRampRate);
}

void FlywheelSubsystem::Periodic() {
    SmartDashboard::PutNumber("RPM_Setpoint", m_setpoint);

    CalculateRPM();
}

void FlywheelSubsystem::SetRPM(double setpoint) {
  m_setpoint = setpoint;
}

double FlywheelSubsystem::CalculateRPM() {
  m_setpoint = m_calculations.GetInitRPMS().to<double>();
  return m_setpoint;
}

void FlywheelSubsystem::StartFiring() {
  double voltage = m_setpoint / 1000;

  m_flywheelmotor.SetVoltage(units::volt_t (voltage));
}