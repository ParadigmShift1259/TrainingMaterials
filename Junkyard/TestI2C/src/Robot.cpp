/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc2/Timer.h>

Robot::Robot()
  : m_mux(I2C::kOnboard, 0x70)
  , m_reAddresser(I2C::kOnboard, 0x29)
{
}

void Robot::MuxSelect(uint8_t muxPort)
{
  if (muxPort > 7)
  {
    frc::DriverStation::ReportError("Specify a mux port between 0 and 7");
    return;
  }
 
  uint8_t muxPortBit[2];
  muxPortBit[0] = 1;// << muxPort;
  muxPortBit[1] = 0;
  //uint8_t muxPortBit = 128 >> muxPort;
  //bool bXferAborted = m_mux.WriteBulk(&muxPortBit[0], 1);
  bool bXferAborted = m_mux.Write(muxPortBit[0], muxPortBit[1]);
  //int32_t result = HAL_WriteI2C(I2C::kOnboard, 0x70, &muxPortBit[0], 1);

  char buf[100];
  sprintf(buf, "Mux select %d bXferAborted = %d", muxPort, bXferAborted);
  //sprintf(buf, "Mux select %d bit mask 0x%08X result = %d", muxPort, muxPortBit[0], result);
  frc::DriverStation::ReportError(buf);

  // bXferAborted = m_mux.AddressOnly();
  // sprintf(buf, "Mux AddressOnly bXferAborted = %d", bXferAborted);
  // frc::DriverStation::ReportError(buf);

  // uint8_t i2cbuf[10];
  // memset(i2cbuf, 0, 10);
  // bXferAborted = m_mux.ReadOnly(1, i2cbuf);
  // sprintf(buf, "Mux select readback %d = %d bXferAborted = %d", muxPort, i2cbuf[0], bXferAborted);
  // frc::DriverStation::ReportError(buf);
}

void Robot::RobotInit()
{
  frc::DriverStation::ReportError("RobotInit()");

  //m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  //m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  //MuxSelect(2);
  //m_colorsensor = new ColorSensorV3(I2C::kOnboard);

  m_out1 = new DigitalOutput(0);
  m_out2 = new DigitalOutput(1);
  
  // Turn all off first
  m_out1->Set(false);
  m_out2->Set(false);

  for (int i = 0; i < 1000; i++); // Temp spin  loop

  // Turn one on at a time
  m_out2->Set(true);
  for (int i = 0; i < 1000; i++); // Temp spin  loop

  bool bXferAborted = m_reAddresser.AddressOnly();
  char buf[100];
  sprintf(buf, "Re-address AddressOnly bXferAborted = %d", bXferAborted);
  frc::DriverStation::ReportError(buf);

  // Re-address
  // uint8_t reg = 0x8a;
  // uint8_t newAddr = 0x54; // new addr 0x54 / 2 = 0x2A
  // uint8_t muxPortBit[3];
  // //muxPortBit[0] = 0x00;
  // muxPortBit[0] = reg;
  // muxPortBit[1] = newAddr;
  // bXferAborted = m_reAddresser.WriteBulk(&muxPortBit[0], 2);
  // //bool bXferAborted = m_reAddresser.Write(reg, newAddr);
  // sprintf(buf, "Re-address bXferAborted = %d", bXferAborted);
  // frc::DriverStation::ReportError(buf);

  //MuxSelect(0);
  m_distsensor1 = new Rev2mDistanceSensorEx(rev::Rev2mDistanceSensorEx::Port::kOnboard
                                          , rev::Rev2mDistanceSensorEx::DistanceUnit::kInches
                                          , rev::Rev2mDistanceSensorEx::RangeProfile::kDefault
                                          , 0x54);

   m_distsensor1->SetAutomaticMode(true);
   m_distsensor1->SetEnabled(true);
  
  //MuxSelect(0);
  //m_distsensor2 = new Rev2mDistanceSensor(rev::Rev2mDistanceSensor::Port::kOnboard, rev::Rev2mDistanceSensor::DistanceUnit::kInches);
  //m_distsensor2->SetAutomaticMode(true);
  //m_distsensor2->SetEnabled(true);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{
  //MuxSelect(2);
  //double IR = m_colorsensor->GetIR();
  // Color detectedColor = m_colorsensor->GetColor();
  // frc::SmartDashboard::PutNumber("Color R", detectedColor.red);
  // frc::SmartDashboard::PutNumber("Color G", detectedColor.green);
  // frc::SmartDashboard::PutNumber("Color B", detectedColor.blue);

  //MuxSelect(0);
  ReadDistance(*m_distsensor1, 1);

  //MuxSelect(0);
  //ReadDistance(*m_distsensor2, 2);
}

void Robot::ReadDistance(Rev2mDistanceSensorEx& distSensor, int sensorNum) 
{
  bool isValid = distSensor.IsRangeValid();

  char buf[100];
  sprintf(buf, "Data %d Valid", sensorNum);
  frc::SmartDashboard::PutBoolean(buf, isValid);
  sprintf(buf, "Data %d Valid %d", sensorNum, isValid);
  frc::DriverStation::ReportError(buf);

  if (isValid)
  {
    /**
     * The current measured range is returned from GetRange(). By default
     * this range is returned in inches.
     */
    sprintf(buf, "Distance %d (in)", sensorNum);
    auto dist = distSensor.GetRange();
    frc::SmartDashboard::PutNumber(buf, dist);
    sprintf(buf, "Distance %d (in) %.3f", sensorNum, dist);
    frc::DriverStation::ReportError(buf);

    /**
     * The timestamp of the last valid measurement (measured in seconds since 
     * the program started), is returned by GetTimestamp().
     */
    sprintf(buf, "Timestamp %d", sensorNum);
    auto ts = distSensor.GetTimestamp();
    frc::SmartDashboard::PutNumber(buf, ts);
    sprintf(buf, "Timestamp %d (in) %.3f", sensorNum, ts);
    frc::DriverStation::ReportError(buf);
  }
  else 
  {
    sprintf(buf, "Distance %d (in)", sensorNum);
    frc::SmartDashboard::PutNumber(buf, -1);
    sprintf(buf, "Timestamp %d", sensorNum);
    frc::SmartDashboard::PutNumber(buf, distSensor.GetTimestamp());
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
