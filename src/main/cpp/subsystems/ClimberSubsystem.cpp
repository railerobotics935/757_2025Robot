// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"
#include "Constants.h"
#include <networktables/NetworkTableInstance.h>

using namespace ClimberConstants;

ClimberSubsystem::ClimberSubsystem() {

  // Burn flash only if desired - true set in constants
  #ifdef BURNELEVATORSPARKMAX
  // Restore defaults
rev::spark::SparkMaxConfig climberSparkMaxConfig{};

  climberSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(kClimberMotorIdleMode)
  .SmartCurrentLimit(kClimberMotorCurrentLimit.value());

  climberSparkMaxConfig.encoder
  .PositionConversionFactor(kClimberPositionFactor);

  m_climberSparkMax.Configure(climberSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);  
  m_climberSparkMax.SetInverted(true); 

 // m_climberSparkMax.BurnFlash();

  std::cout << "Flash Burned on climber subsystem\r\n";
  #else
  std::cout << "Flash was not burned on climber subsystem\r\n";
  #endif

  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  
  auto nt_table = nt_inst.GetTable("Climber");

  //m_ClimberLimitSwitch = nt_table->GetEntry("Climber/Limit Switch");
  m_ClimberDistance = nt_table->GetEntry("Climber/Distance Extended");
 
    // Set the distance per pulse if needed
 // m_climberEncoder.SetDistancePerPulse(0.02 / 360.0); // Example for a 360 PPR encoder
  
}

/*bool ClimberSubsystem::ClimberAtBase() {
  return !m_LimitSwitch.Get();
}*/

void ClimberSubsystem::Periodic() {
  UpdateNTE();

  /*if (ClimberAtBase())
   m_climberEncoder.SetPosition(0.0);*/
}

void ClimberSubsystem::UpdateNTE() {
  //m_ClimberLimitSwitch.SetBoolean(ClimberAtBase());
  m_ClimberDistance.SetDouble(m_climberEncoder.GetPosition());
}

void ClimberSubsystem::SetClimberPower(double power) {
  /*if (power < 0.0 && m_climberEncoder.GetDistance() < -6.2) {
   m_climberSparkMax.Set(0.0);
  }
  else {
    if (ClimberAtBase() && power > 0.0)
     m_climberSparkMax.Set(0.0);

    else {
     */
    m_climberSparkMax.Set(power);
}
  //}}
  
void ClimberSubsystem::LatchServo(double value) {
 m_climberServo.Set(value);
}