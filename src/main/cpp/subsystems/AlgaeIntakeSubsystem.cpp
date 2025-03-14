// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/AlgaeIntakeSubsystem.h"
#include "Constants.h"

AlgaeIntakeSubsystem::AlgaeIntakeSubsystem() 
: m_rightAlgaeIntakeSparkMax{IntakeConstants::kRightAlgaeIntakeMotorID, IntakeConstants::kMotorType},
  m_leftAlgaeIntakeSparkMax{IntakeConstants::kLeftAlgaeIntakeMotorID, IntakeConstants::kMotorType} {
// Implementation of subsystem constructor goes here.
} 


void AlgaeIntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void AlgaeIntakeSubsystem::SetAlgaeIntakeMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_rightAlgaeIntakeSparkMax.Set(power);
  m_leftAlgaeIntakeSparkMax.Set(power);
}

void AlgaeIntakeSubsystem::SetPitchPosition(units::radian_t setAngle) {
  // Use the Spark MAX internal PID controller to reach the setAngle
  // - downward is counterclockwise along the robot Y-axis => positive direction for Angle
  // - check intake rotates down when applying a positive value to the SetPitchPower method,
  //   if this is not the case, invert the Pitch Spark MAX motor controller.
  // - check absolute encoder on the pitch angle to have an increasing value when rotating downward
  //   if this is not the case, add minus sign to m_pitchAbsoluteEncoder.GetPosition() references.
  m_pitchPIDController.SetReference(setAngle.value(), rev::spark::SparkLowLevel::ControlType::kPosition);

  // Automatically disable control control direction goes outside mechanical operating limits
  // - IntakeConstants::kMinimumAngle sets the upper pitch limit, this is the lowest angle value
  // - IntakeConstants::kMaximumAngle sets the downward pitch limit, this is the highest angle value

  // Limit Pitch going too far up
  if ((m_pitchAbsoluteEncoder.GetPosition() < IntakeConstants::kMinimumAngle) &&
      (m_pitchSparkMax.Get() < 0.0)) {
    m_pitchSparkMax.Set(0.0);
  }

  // Limit Pitch going too far down
  if ((m_pitchAbsoluteEncoder.GetPosition() > IntakeConstants::kMaximumAngle) &&
      (m_pitchSparkMax.Get() > 0.0)) {
    m_pitchSparkMax.Set(0.0);
  }
}

void AlgaeIntakeSubsystem::SetPitchPower(double power) {
  m_pitchSparkMax.Set(power);
}

/*double AlgaeIntakeSubsystem::SignedSquare(double input) {
  if (input > 0) {
    return std::pow(input, 2);
  }
  else {
    return -std::pow(input, 2);
  }
} */

bool AlgaeIntakeSubsystem::AlgaeInIntake() {
  return m_lightSensor.Get();
}

void AlgaeIntakeSubsystem::ConfigureAlgaeSparkMax() {
  // Configure the Intake Spark MAX
  rev::spark::SparkMaxConfig rightAlgaeIntakeSparkMaxConfig{};
  rev::spark::SparkMaxConfig leftAlgaeIntakeSparkMaxConfig{};


  rightAlgaeIntakeSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(IntakeConstants::kIntakeMotorIdleMode)
  .SmartCurrentLimit(IntakeConstants::kAlgaeIntakeMotorCurrentLimit.value())
  .Follow(IntakeConstants::kLeftAlgaeIntakeMotorID, true);

  leftAlgaeIntakeSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(IntakeConstants::kIntakeMotorIdleMode)
  .SmartCurrentLimit(IntakeConstants::kAlgaeIntakeMotorCurrentLimit.value());

  //intakeSparkMaxConfig.encoder
  //.PositionConversionFactor(kIntakePositionFactor)
  //.VelocityConversionFactor(kIntakeVelocityFactor);

  m_rightAlgaeIntakeSparkMax.Configure(rightAlgaeIntakeSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
  m_leftAlgaeIntakeSparkMax.Configure(rightAlgaeIntakeSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

}
  // Configure the Pitch Spark MAX
  //rev::spark::SparkMaxConfig pitchSparkMaxConfig{};

  /*pitchSparkMaxConfig
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
*/