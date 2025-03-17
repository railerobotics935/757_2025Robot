// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/CoralIntakeSubsystem.h"
#include "Constants.h"

CoralIntakeSubsystem::CoralIntakeSubsystem() 

: m_coralIntakeSparkMax{IntakeConstants::kCoralIntakeMotorID, IntakeConstants::kMotorType} {


   #ifdef BURNCORALINTAKESPARKMAX

  rev::spark::SparkMaxConfig coralIntakeSparkMaxConfig{};

  coralIntakeSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(IntakeConstants::kIntakeMotorIdleMode)
  .SmartCurrentLimit(IntakeConstants::kCoralIntakeMotorCurrentLimit.value());

  m_coralIntakeSparkMax.Configure(coralIntakeSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

  std::cout << "Flash Burned on coralintake subsystem\r\n";
  #else
  std::cout << "Flash was not burned on coralintake subsystem\r\n";
  #endif

}

void CoralIntakeSubsystem::SetCoralIntakeMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_coralIntakeSparkMax.Set(power);
}

bool CoralIntakeSubsystem::CoralInIntake() {
  return m_lightSensor.Get();
}

double CoralIntakeSubsystem::GetDirection() {
  return m_coralIntakeSparkMax.Get();
}
