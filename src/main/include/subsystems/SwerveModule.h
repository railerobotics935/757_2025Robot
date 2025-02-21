// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>
#include <frc/AnalogInput.h>                        // for closed-loop control on RoboRIO
#include <frc/controller/PIDController.h>           // for closed-loop control on RoboRIO
#include <frc/controller/ProfiledPIDController.h>   // for closed-loop control on RoboRIO
#include <frc/controller/SimpleMotorFeedforward.h>  // for closed-loop control on robo
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ProfiledPIDController.h>

#include <rev/SparkAbsoluteEncoder.h>               // for through-hole encoder on Spark MAX
#include <rev/SparkClosedLoopController.h>                 // for closed-loop control on Spark MAX 
#include <rev/SparkRelativeEncoder.h>               // for closed-loop control on Spark MAX

#include <rev/SparkMax.h>
//#include "rev/SparkAnalogSensor.h"
#include "ctre/phoenix6/CANCoder.hpp"

#include "Constants.h"

using namespace ctre::phoenix6;

class SwerveModule {
 public:
  SwerveModule(const int drivingCANId, const int turningCANId, const int turnSensorCANId);

  frc::SwerveModuleState GetState();

  frc::SwerveModulePosition GetPosition();

  void SetDesiredState(const frc::SwerveModuleState& state);

  /**
   * Burn configuration onto sparkmax motorcontrollers
  */
  void ConfigureSparkMax();
  
  void SetTurningPID(double Kp, double Ki, double Kd);
  void SetDrivingPID(double Kp, double Ki, double Kd);


 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.
  
  rev::spark::SparkMax m_drivingSparkMax;
  rev::spark::SparkMax m_turningSparkMax;

  rev::spark::SparkRelativeEncoder m_drivingEncoder = m_drivingSparkMax.GetEncoder();
  //rev::spark::SparkAbsoluteEncoder m_turningAbsoluteEncoder = m_turningSparkMax.GetAbsoluteEncoder();
  hardware::CANcoder m_turningAbsoluteEncoder;

  rev::spark::SparkClosedLoopController m_drivingPIDController = m_drivingSparkMax.GetClosedLoopController();
  

  frc::SwerveModuleState m_desiredState{units::meters_per_second_t{0.0},
                                        frc::Rotation2d()};

  static constexpr auto kModuleMaxAngularVelocity = 18.0 * std::numbers::pi * 1_rad_per_s;  // radians per second
  static constexpr auto kModuleMaxAngularAcceleration = 8.0 * std::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2

  frc::ProfiledPIDController<units::radians> m_turningPIDController{
//      12.5, 190.0, 0.15,
      ModuleConstants::kTurningP, ModuleConstants::kTurningI, ModuleConstants::kTurningD,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

  frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{0.5_V, 0.5_V / 1_rad_per_s};

};
