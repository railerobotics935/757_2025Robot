// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>

#include <rev/config/SparkMaxConfig.h>

#include <frc/controller/PIDController.h>

#include "Constants.h"

using namespace ModuleConstants;

SwerveModule::SwerveModule(const int drivingCANId, const int turningCANId,
                               const double turningEncoderOffset)
    : m_drivingSparkMax(drivingCANId, rev::spark::SparkMax::MotorType::kBrushless),
      m_turningSparkMax(turningCANId, rev::spark::SparkMax::MotorType::kBrushless) {

  #ifdef BURNMODULESPARKMAX 
  ConfigureSparkMax();
  std::cout << "Flash Burned on Swerve Module\r\n";
  #else
  std::cout << "Flash was not burned on Swerve Module\r\n";
  #endif

  m_turningEncoderOffset = turningEncoderOffset;
  m_desiredState.angle =
      frc::Rotation2d(units::radian_t{m_turningAbsoluteEncoder.GetPosition()});
  m_drivingEncoder.SetPosition(0);
}

void SwerveModule::ConfigureSparkMax() {
  // Factory reset, so we get the SPARKS MAX to a known state before configuring
  // them. This is useful in case a SPARK MAX is swapped out.
  rev::spark::SparkMaxConfig driveSparkMaxConfig{};

  driveSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(kDrivingMotorIdleMode)
  .SmartCurrentLimit(kDrivingMotorCurrentLimit.value());

  driveSparkMaxConfig.absoluteEncoder
  .PositionConversionFactor(kDrivingEncoderPositionFactor)
  .VelocityConversionFactor(kDrivingEncoderVelocityFactor);

  driveSparkMaxConfig.closedLoop
  .Pidf(kDrivingP, kDrivingI, kDrivingD, kDrivingFF)
  .OutputRange(kDrivingMinOutput, kDrivingMaxOutput);

  m_drivingSparkMax.Configure(driveSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
  

  rev::spark::SparkMaxConfig turningSparkMaxConfig{};

  turningSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(kTurningMotorIdleMode)
  .SmartCurrentLimit(kTurningMotorCurrentLimit.value());

  turningSparkMaxConfig.absoluteEncoder
  .PositionConversionFactor(kTurningEncoderPositionFactor)
  .VelocityConversionFactor( kTurningEncoderVelocityFactor)
  .Inverted(kTurningEncoderInverted);

  turningSparkMaxConfig.closedLoop
  .Pidf(kTurningP, kTurningI, kTurningD, kTurningFF)
  .OutputRange(kTurningMinOutput, kTurningMaxOutput)
  .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
  .PositionWrappingEnabled(true)
  .PositionWrappingMinInput(kTurningEncoderPositionPIDMinInput.value())
  .PositionWrappingMaxInput(kTurningEncoderPositionPIDMaxInput.value());

  m_turningSparkMax.Configure(turningSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
  

        
  // Set the PID Controller to use the duty cycle encoder on the swerve
  // module instead of the built in NEO550 encoder.
  


}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_drivingEncoder.GetVelocity()},
          units::radian_t{m_turningAbsoluteEncoder.GetPosition() - m_turningEncoderOffset}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {units::meter_t{m_drivingEncoder.GetPosition()},
          units::radian_t{m_turningAbsoluteEncoder.GetPosition() - m_turningEncoderOffset}};
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& desiredState){
  // Apply chassis angular offset to the desired state.
  frc::SwerveModuleState correctedDesiredState{};
  correctedDesiredState.speed = desiredState.speed;
  correctedDesiredState.angle =
      desiredState.angle +
      frc::Rotation2d(units::radian_t{m_turningEncoderOffset});

  // Optimize the reference state to avoid spinning further than 90 degrees.
  frc::SwerveModuleState optimizedDesiredState{/*frc::SwerveModuleState::Optimize(
      correctedDesiredState, frc::Rotation2d(units::radian_t{
                                 m_turningAbsoluteEncoder.GetPosition()}))*/};

  // Command driving and turning SPARKS MAX towards their respective setpoints.
  m_drivingPIDController.SetReference((double)optimizedDesiredState.speed,
                                      rev::spark::SparkMax::ControlType::kVelocity);
  m_turningPIDController.SetReference(
      optimizedDesiredState.angle.Radians().value(),
      rev::spark::SparkMax::ControlType::kPosition);

  m_desiredState = desiredState;
}

