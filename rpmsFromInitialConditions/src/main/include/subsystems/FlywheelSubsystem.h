#pragma once

#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>

#include "units/length.h"
#include "units/time.h"
#include "units/voltage.h"
#include <frc/controller/SimpleMotorFeedforward.h>

#include "Constants.h"

using namespace rev;
using namespace std;
using namespace frc;

class FlywheelSubsystem : public frc2::SubsystemBase
{
public:
    FlywheelSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Sets the flywheel to a desired rpm
    /// \param rpm         Desired set rpm
    void SetRPM(double rpm);
    /// \return The current desired RPM of the flywheel
    double GetRPM();

    /// \return Whether or not the flywheel in the zone to use the alternative "maintain" PID values
    bool IsAtMaintainPID();
    /// \return Whether or not the flywheel is at the desired RPM
    bool IsAtRPM();
    /// \return Whether or not the flywheel is at the desired RPM, but only positive side
    bool IsAtRPMPositive();

protected:
    /// Calculates the next desired RPM for the flywheel
    void CalculateRPM();

private:
    /// \name Flywheel shooter
    /// NEO that runs shooter, maintains set RPM with the PID, encoder, and feedforward to convert rpm directly to power
    ///@{
    CANSparkMax m_flywheelmotor;
    CANSparkMax m_followerFlywheelMotor;
    SparkMaxPIDController m_flywheelPID = m_flywheelmotor.GetPIDController();
    SparkMaxRelativeEncoder m_flywheelencoder = m_flywheelmotor.GetEncoder();
    SimpleMotorFeedforward<units::meters> m_flywheelFF;
    ///@}

    /// Current desired setpoint of the flywheel in RPM
    double m_setpoint;
};