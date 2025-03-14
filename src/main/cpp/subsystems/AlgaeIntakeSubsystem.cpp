// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/AlgaeIntakeSubsystem.h"
#include "Constants.h"

AlgaeIntakeSubsystem::AlgaeIntakeSubsystem() 

: m_rightAlgaeIntakeSparkMax{IntakeConstants::kRightAlgaeIntakeMotorID, IntakeConstants::kMotorType},
  m_leftAlgaeIntakeSparkMax{IntakeConstants::kLeftAlgaeIntakeMotorID, IntakeConstants::kMotorType},
  m_rightPitchSparkMax{IntakeConstants::kRightAlgaePitchMotorID, IntakeConstants::kMotorType},
  m_leftPitchSparkMax{IntakeConstants::kLeftAlgaePitchMotorID, IntakeConstants::kMotorType}
   {

  #ifdef BURNALGAEINTAKESPARKMAX
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

  m_rightAlgaeIntakeSparkMax.Configure(rightAlgaeIntakeSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
  m_leftAlgaeIntakeSparkMax.Configure(leftAlgaeIntakeSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
  
  std::cout << "Flash Burned on algaeintake subsystem\r\n";
  #else
  std::cout << "Flash was not burned on algaeintake subsystem\r\n";
  #endif

  #ifdef BURNALGAEPITCHSPARKMAX

  rev::spark::SparkMaxConfig rightPitchSparkMaxConfig{};
  rev::spark::SparkMaxConfig leftPitchSparkMaxConfig{};

  rightPitchSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(IntakeConstants::kIntakeMotorIdleMode)
  .SmartCurrentLimit(IntakeConstants::kAlgaeIntakeMotorCurrentLimit.value())
  .Follow(IntakeConstants::kLeftAlgaePitchMotorID, true);

  leftPitchSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(IntakeConstants::kIntakeMotorIdleMode)
  .SmartCurrentLimit(IntakeConstants::kAlgaeIntakeMotorCurrentLimit.value());

  m_rightPitchSparkMax.Configure(rightPitchSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
  m_leftPitchSparkMax.Configure(leftPitchSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
  std::cout << "Flash Burned on algaeintake subsystem\r\n";
  #else
  std::cout << "Flash was not burned on algaeintake subsystem\r\n";
  #endif

} 

bool AlgaeIntakeSubsystem::RightAlgaeIntakeAtBase() {
  return !m_RightAlgaeLimitSwitch.Get();
}

bool AlgaeIntakeSubsystem::LeftAlgaeIntakeAtBase() {
  return !m_LeftAlgaeLimitSwitch.Get();
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
  m_rightPitchPIDController.SetReference(setAngle.value(), rev::spark::SparkLowLevel::ControlType::kPosition);

  // Automatically disable control control direction goes outside mechanical operating limits
  // - IntakeConstants::kMinimumAngle sets the upper pitch limit, this is the lowest angle value
  // - IntakeConstants::kMaximumAngle sets the downward pitch limit, this is the highest angle value

  // Limit Pitch going too far up
 
}

void AlgaeIntakeSubsystem::SetPitchPower(double power) {
  m_rightPitchSparkMax.Set(power);
}

void AlgaeIntakeSubsystem::ConfigureAlgaeSparkMax() {
  // Configure the Intake Spark MAX
  
}