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
                               const int turnSensorCANId)
    : m_drivingSparkMax(drivingCANId, rev::spark::SparkMax::MotorType::kBrushless),
      m_turningSparkMax(turningCANId, rev::spark::SparkMax::MotorType::kBrushless),
      m_turningAbsoluteEncoder(turnSensorCANId) {

  #ifdef BURNMODULESPARKMAX 
  ConfigureSparkMax();
  std::cout << "Flash Burned on Swerve Module\r\n";
  #else
  std::cout << "Flash was not burned on Swerve Module\r\n";
  #endif

  m_desiredState.angle =
      frc::Rotation2d(units::radian_t{m_turningAbsoluteEncoder.GetAbsolutePosition().GetValueAsDouble()});
  m_drivingEncoder.SetPosition(0);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  // if motor is misbehaving, check turning direction
  m_drivingSparkMax.SetInverted(true);
  m_turningSparkMax.SetInverted(true);
  m_turningPIDController.EnableContinuousInput(-units::radian_t(std::numbers::pi), units::radian_t(std::numbers::pi));
}

void SwerveModule::ConfigureSparkMax() {
  // Factory reset for driving SPARK MAX, so we get the SPARKS MAX to a known state before configuring
  // them. This is useful in case a SPARK MAX is swapped out.
  rev::spark::SparkMaxConfig driveSparkMaxConfig{};

  driveSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(kDrivingMotorIdleMode)
  .SmartCurrentLimit(kDrivingMotorCurrentLimit.value());

  driveSparkMaxConfig.encoder
  .PositionConversionFactor(kDrivingEncoderPositionFactor)
  .VelocityConversionFactor(kDrivingEncoderVelocityFactor);

  driveSparkMaxConfig.closedLoop
  .Pidf(kDrivingP, kDrivingI, kDrivingD, kDrivingFF)
  .OutputRange(kDrivingMinOutput, kDrivingMaxOutput);

  m_drivingSparkMax.Configure(driveSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
  
  // Factory reset for turning SPARK MAX, so we get the SPARKS MAX to a known state before configuring
  // them. This is useful in case a SPARK MAX is swapped out.
  rev::spark::SparkMaxConfig turningSparkMaxConfig{};

  turningSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(kTurningMotorIdleMode)
  .SmartCurrentLimit(kTurningMotorCurrentLimit.value());

  turningSparkMaxConfig.absoluteEncoder
  .PositionConversionFactor(kTurningEncoderPositionFactor)
  .VelocityConversionFactor(kTurningEncoderVelocityFactor);
//  .Inverted(kTurningEncoderInverted);

#if 0
  turningSparkMaxConfig.closedLoop
  .Pidf(kTurningP, kTurningI, kTurningD, kTurningFF)
  .OutputRange(kTurningMinOutput, kTurningMaxOutput)
  .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
  .PositionWrappingEnabled(true)
  .PositionWrappingMinInput(kTurningEncoderPositionPIDMinInput.value())
  .PositionWrappingMaxInput(kTurningEncoderPositionPIDMaxInput.value());
#endif

  m_turningSparkMax.Configure(turningSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
  
       
  // Set the PID Controller to use the duty cycle encoder on the swerve
  // module instead of the built in NEO550 encoder.
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_drivingEncoder.GetVelocity()},
          units::radian_t{m_turningAbsoluteEncoder.GetAbsolutePosition().GetValueAsDouble() * 2 * std::numbers::pi}};
}
//Gets the current position of the swerve module.
frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {units::meter_t{m_drivingEncoder.GetPosition()},
          units::radian_t{m_turningAbsoluteEncoder.GetAbsolutePosition().GetValueAsDouble() * 2 * std::numbers::pi}};
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& desiredState){

  // Optimize the reference state to avoid spinning further than 90 degrees.
  frc::SwerveModuleState optimizedDesiredState{frc::SwerveModuleState::Optimize(desiredState,
   units::radian_t(m_turningAbsoluteEncoder.GetAbsolutePosition().GetValueAsDouble() * 2 * std::numbers::pi))};

  // Command driving and turning SPARKS MAX towards their respective setpoints.
  m_drivingPIDController.SetReference((double)optimizedDesiredState.speed,
                                      rev::spark::SparkMax::ControlType::kVelocity);

  // PID Controller in Roborio
  const auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t(m_turningAbsoluteEncoder.GetAbsolutePosition().GetValueAsDouble() * 2 * std::numbers::pi), (optimizedDesiredState.angle).Radians());

  const auto turnFeedforward = m_turnFeedforward.Calculate(m_turningPIDController.GetSetpoint().velocity);

  m_turningSparkMax.SetVoltage(units::volt_t{turnOutput});

  m_desiredState = desiredState;
} 

void SwerveModule::SetTurningPID(double Kp, double Ki, double Kd) {
  m_turningPIDController.SetPID(Kp, Ki, Kd);
}

void SwerveModule::SetDrivingPID(double Kp, double Ki, double Kd) {
//#if 0
  rev::spark::SparkMaxConfig driveSparkMaxConfig{};

  driveSparkMaxConfig.closedLoop
    .Pidf(Kp, Ki, Kd, kDrivingFF)
    .OutputRange(kDrivingMinOutput, kDrivingMaxOutput);

  m_drivingSparkMax.Configure(driveSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
//#endif
}
