#pragma once

#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>

#include "units/length.h"
#include "units/time.h"
#include "units/voltage.h"
#include <frc/controller/SimpleMotorFeedforward.h>

#include "Calculations.h"
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

    void StartFiring();

protected:
    /// Calculates the next desired RPM for the flywheel
    double CalculateRPM();

private:
    /// \name Flywheel shooter
    /// NEO that runs shooter, maintains set RPM with the PID, encoder, and feedforward to convert rpm directly to power
    ///@{
    CANSparkMax m_flywheelmotor;
    ///@}

    Calculations m_calculations;

    /// Current desired setpoint of the flywheel in RPM
    double m_setpoint;
};