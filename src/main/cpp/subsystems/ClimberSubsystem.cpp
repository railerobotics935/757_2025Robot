// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"
#include "Constants.h"
#include <networktables/NetworkTableInstance.h>

using namespace ClimberConstants;

ClimberSubsystem::ClimberSubsystem() {

  // Burn flash only if desired - true set in constants
  #ifdef BURNCLIMBERSPARKMAX
  // Restore defaults
rev::spark::SparkMaxConfig climberSparkMaxConfig{};

  climberSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(kClimberMotorIdleMode)
  .SmartCurrentLimit(kClimberMotorCurrentLimit.value())
  .Inverted(true);

  climberSparkMaxConfig.encoder
  .PositionConversionFactor(kClimberPositionFactor);

  m_climberSparkMax.Configure(climberSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters); 

 // m_climberSparkMax.BurnFlash();

  std::cout << "Flash Burned on climber subsystem\r\n";
  #else
  std::cout << "Flash was not burned on climber subsystem\r\n";
  #endif

  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  
  auto nt_table = nt_inst.GetTable("Climber");

  m_ClimberDistance = nt_table->GetEntry("Climber/Distance Extended");

}

void ClimberSubsystem::Periodic() {
  UpdateNTE();

}

void ClimberSubsystem::UpdateNTE() {

  m_ClimberDistance.SetDouble(m_climberEncoder.GetPosition());
}

void ClimberSubsystem::SetClimberPower(double power) {

  m_climberSparkMax.Set(power);
}

  
