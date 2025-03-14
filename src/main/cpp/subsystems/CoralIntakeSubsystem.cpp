// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralIntakeSubsystem.h"
#include "Constants.h"

CoralIntakeSubsystem::CoralIntakeSubsystem() 
: m_coralIntakeSparkMax{IntakeConstants::kCoralIntakeMotorID, IntakeConstants::kMotorType}, 
  m_coralPitchSparkMax{IntakeConstants::kCoralPitchMotorID, IntakeConstants::kMotorType} {
  // Implementation of subsystem constructor goes here.
}

void CoralIntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void CoralIntakeSubsystem::SetCoralIntakeMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_coralIntakeSparkMax.Set(power);
}

void CoralIntakeSubsystem::SetCoralPitchPosition(units::radian_t setAngle) {
  // Use the Spark MAX internal PID controller to reach the setAngle
  // - downward is counterclockwise along the robot Y-axis => positive direction for Angle
  // - check intake rotates down when applying a positive value to the SetPitchPower method,
  //   if this is not the case, invert the Pitch Spark MAX motor controller.
  // - check absolute encoder on the pitch angle to have an increasing value when rotating downward
  //   if this is not the case, add minus sign to m_pitchAbsoluteEncoder.GetPosition() references.
  m_coralPitchPIDController.SetReference(setAngle.value(), rev::spark::SparkLowLevel::ControlType::kPosition);

  // Automatically disable control control direction goes outside mechanical operating limits
  // - IntakeConstants::kMinimumAngle sets the upper pitch limit, this is the lowest angle value
  // - IntakeConstants::kMaximumAngle sets the downward pitch limit, this is the highest angle value

  // Limit Pitch going too far up
  if ((m_coralPitchAbsoluteEncoder.GetPosition() > IntakeConstants::kMinimumAngle) &&
      (m_coralPitchSparkMax.Get() < 0.0)) {
    m_coralPitchSparkMax.Set(0.0);
  }

  // Limit Pitch going too far down
  if ((m_coralPitchAbsoluteEncoder.GetPosition() < IntakeConstants::kMaximumAngle) &&
      (m_coralPitchSparkMax.Get() > 0.0)) {
    m_coralPitchSparkMax.Set(0.0);
  }
}

void CoralIntakeSubsystem::SetCoralPitchPower(double power) {
  m_coralPitchSparkMax.Set(power);
}
/*
double CoralIntakeSubsystem::SignedSquare(double input) {
  if (input > 0) {
    return std::pow(input, 2);
  }
  else {
    return -std::pow(input, 2);
  }
}
*/

bool CoralIntakeSubsystem::CoralInIntake() {
  return m_lightSensor.Get();
}

void CoralIntakeSubsystem::ConfigureSparkMax() {
  // Configure the Intake Spark MAX
  rev::spark::SparkMaxConfig coralIntakeSparkMaxConfig{};

  coralIntakeSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(IntakeConstants::kIntakeMotorIdleMode)
  .SmartCurrentLimit(IntakeConstants::kCoralIntakeMotorCurrentLimit.value());

  //intakeSparkMaxConfig.encoder
  //.PositionConversionFactor(kIntakePositionFactor)
  //.VelocityConversionFactor(kIntakeVelocityFactor);

  m_coralIntakeSparkMax.Configure(coralIntakeSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

  // Configure the Pitch Spark MAX
  rev::spark::SparkMaxConfig coralPitchSparkMaxConfig{};

  coralPitchSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(IntakeConstants::kIntakeMotorIdleMode)
  .SmartCurrentLimit(IntakeConstants::kCoralIntakeMotorCurrentLimit.value());

  coralPitchSparkMaxConfig.closedLoop
  .Pidf(IntakeConstants::kPitchP, IntakeConstants::kPitchI, IntakeConstants::kPitchD, IntakeConstants::kPitchFF)
  .OutputRange(IntakeConstants::kPitchMinOutput, IntakeConstants::kPitchMaxOutput)
  .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder);

  m_coralPitchSparkMax.Configure(coralPitchSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}

double CoralIntakeSubsystem::GetDirection() {
  return m_coralPitchSparkMax.Get();
}
