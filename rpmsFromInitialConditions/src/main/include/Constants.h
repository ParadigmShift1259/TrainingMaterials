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

#pragma once

constexpr auto gravity = units::meters_per_second_squared_t(32.0);

constexpr units::kilogram_t flywheelMass = units::pound_t(5.0);
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

constexpr int kMotorPort = 20;
constexpr double kRampRate = 1.0;

constexpr double kflywheelGearRatio = 30 / 20;

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
