// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/moment_of_inertia.h>
#include <units/mass.h>
#include <units/length.h>
#include <units/dimensionless.h>
#include <units/angular_velocity.h>
#include <units/energy.h>

#include <wpi/numbers>

#pragma once

constexpr auto gravity = units::meters_per_second_squared_t(9.81);

constexpr units::kilogram_t flywheelMass = units::pound_t(4.7);
constexpr units::meter_t flywheelRadius = 2.0_in;
constexpr auto flywheelRotInertia =  0.5 * flywheelMass * flywheelRadius * flywheelRadius;

constexpr units::kilogram_t cargoMass = units::ounce_t(9.5);
constexpr units::meter_t cargoRadius = units::inch_t(4.75);
constexpr auto cargoRotInertia = 2.0 / 3.0 * cargoMass * cargoRadius * cargoRadius;

constexpr auto rotInertiaRatio = flywheelRotInertia / cargoRotInertia;

constexpr units::dimensionless_t linearRegSlope = 0.874;
constexpr auto linearRegConst = units::radians_per_second_t (12.7);

// auto test = flywheelRotInertia * radians_per_second_squared_t(10.0) /*/ radian_t(10.0)*/;

constexpr int kPrimaryControllerPort = 0;

namespace FlywheelConstants
{
    constexpr double kPrimaryMotorPort = 1;     //!< Flywheel CAN ID (Primary SparkMAX)
    constexpr double kFollowerMotorPort = 2;            //!< Flywheel CAN ID (Following SparkMAX)

    constexpr double kRampRate = 1.0;
    // Total error allowed for the flywheel, in RPM
    constexpr double kAllowedError = 75;
    constexpr double kMaintainPIDError = 300;

    // General multiplier added, adjusts for ball conditions and general firing
    constexpr double kHomingRPMMultiplier = 1.0175;
    constexpr double kIdleHomingRPMMultiplier = 1.01;
    // Additional multiplier applied to flywheel speed while firing 
    // Ensures all ball trajectories are straight
    constexpr double kFiringRPMMultiplier = 1.01; //TEMP 1.015; //2; //1.035; //1.05;

    // Launch PID values, used to first get to setpoint
    constexpr double kP = 0.0002900;
    constexpr double kI = 0;
    constexpr double kD = 0;

    // Maintain PID values, used to adjust for error once the robot is shooting
    constexpr double kMP = 0.002000;
    constexpr double kMI = 0.00000001;
    constexpr double kMD = 0.000001;

    constexpr double kMinOut = 0;
    constexpr double kMaxOut = 1.0;

    constexpr double kS = 0.26625;  // Characterization should be repeated with 2 Neos
    constexpr double kV = 0.12771;
    constexpr double kA = 0.031171;

    // Diameter is in meters
    constexpr double kWheelDiameter = 0.1016;   // 4 inches
    constexpr double kSecondsPerMinute = 60;
    constexpr double kWheelMetersPerRev = kWheelDiameter * wpi::numbers::pi;
    // Meters per second to Revolutions per minute
    constexpr double kMPSPerRPM = kWheelMetersPerRev / kSecondsPerMinute;
    constexpr double kWheelRevPerMotorRev = 1.3;

    /// Use MPSPerRPM to determine the ramp rates, current values are just placeholders
    constexpr double kIdleRPM = 2000;
    /// The fixed RPM to fire at the trench given very heavy defense
    constexpr double kTrenchRPM = 3000;

    /// One turn of the Neo is 1.33 turns of the Flywheel
    constexpr double kGearRatio = 3.0 / 2.0;
}

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
