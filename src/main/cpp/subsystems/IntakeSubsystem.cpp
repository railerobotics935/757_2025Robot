// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"


IntakeSubsystem::IntakeSubsystem() 
: m_intakeSparkMax{IntakeConstants::kIntakeMotorID, IntakeConstants::kMotorType}, 
  m_pitchSparkMax{IntakeConstants::kPitchMotorID, IntakeConstants::kMotorType} {
  // Implementation of subsystem constructor goes here.
  #ifdef BURNINTAKESPARKMAX
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
  .OutputRange(IntakeConstants::kPitchMinOutput, IntakeConstants::kPitchMaxOutput);
  

  m_pitchSparkMax.Configure(pitchSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);


  std::cout << "Burned Intake Motor Controller\r\n";
  #else
  std::cout << "Did Not Burn Intake Motor Controller\r\n";
  #endif
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
  if( units::radian_t(m_pitchAbsoluteEncoder.GetPosition()) != setAngle) { 
  m_pitchSparkMax.Set(1);
  }
  else {
    m_pitchSparkMax.Set(0);
  }
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