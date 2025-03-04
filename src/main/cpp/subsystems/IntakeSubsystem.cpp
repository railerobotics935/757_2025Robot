// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"


IntakeSubsystem::IntakeSubsystem() 
: m_intakeSparkMax{IntakeConstants::kIntakeMotorID, IntakeConstants::kMotorType}, 
  m_pitchSparkMax{IntakeConstants::kPitchMotorID, IntakeConstants::kMotorType} {
  // Implementation of subsystem constructor goes here.
}

void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void IntakeSubsystem::SetIntakeMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_intakeSparkMax.Set(power);
}

void IntakeSubsystem::SetPitchPosition(units::radian_t setAngle) {
  // Sets the motor's power (between -1.0 and 1.0).
  m_pitchPIDController.SetReference(setAngle.value(), rev::spark::SparkLowLevel::ControlType::kPosition);

  // automatically disable control when outside mechanical operating limits
  
  // TODO: split in the 2 directions, check motor power and only disable when going past a boundary

  if ((m_pitchAbsoluteEncoder.GetPosition() < IntakeConstants::kMinimumAngle) || 
      (m_pitchAbsoluteEncoder.GetPosition() > IntakeConstants::kMaximumAngle))
  {
    m_pitchSparkMax.Set(0.0);
  }
}

void IntakeSubsystem::SetPitchPower(double power) {
  m_pitchSparkMax.Set(power);
}

double IntakeSubsystem::SignedSquare(double input) {
  if (input > 0) {
    return std::pow(input, 2);
  }
  else {
    return -std::pow(input, 2);
  }
}

bool IntakeSubsystem::CoralInIntake() {
  return m_lightSensor.Get();
}

void IntakeSubsystem::ConfigureSparkMax() {
  rev::spark::SparkMaxConfig intakeSparkMaxConfig{};

  intakeSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(IntakeConstants::kIntakeMotorIdleMode)
  .SmartCurrentLimit(IntakeConstants::kIntakeMotorCurrentLimit.value());

  //intakeSparkMaxConfig.encoder
  //.PositionConversionFactor(kIntakePositionFactor)
  //.VelocityConversionFactor(kIntakeVelocityFactor);

  m_intakeSparkMax.Configure(intakeSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

rev::spark::SparkMaxConfig pitchSparkMaxConfig{};

  pitchSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(IntakeConstants::kIntakeMotorIdleMode)
  .SmartCurrentLimit(IntakeConstants::kIntakeMotorCurrentLimit.value());

  pitchSparkMaxConfig.closedLoop
  .Pidf(IntakeConstants::kPitchP, IntakeConstants::kPitchI, IntakeConstants::kPitchD, IntakeConstants::kPitchFF)
  .OutputRange(IntakeConstants::kPitchMinOutput, IntakeConstants::kPitchMaxOutput)
  .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder);

  m_pitchSparkMax.Configure(pitchSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}

double IntakeSubsystem::GetDirection() {
  return m_pitchSparkMax.Get();
}
