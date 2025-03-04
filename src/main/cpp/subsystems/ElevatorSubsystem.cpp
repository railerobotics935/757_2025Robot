// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"
#include "Constants.h"
#include <networktables/NetworkTableInstance.h>

using namespace ElevatorConstants;

ElevatorSubsystem::ElevatorSubsystem() {

  // Burn flash only if desired - true set in constants
  #ifdef BURNELEVATORSPARKMAX
  // Restore defaults
rev::spark::SparkMaxConfig elevatorSparkMaxConfig{};

  elevatorSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
  .SetIdleMode(kElevatorMotorIdleMode)
  .SmartCurrentLimit(kElevatorMotorCurrentLimit.value());

  elevatorSparkMaxConfig.encoder
  .PositionConversionFactor(kElevatorPositionFactor)
  .VelocityConversionFactor(kElevatorVelocityFactor);

  std::cout << "drive encoder velocity factor: " << kElevatorVelocityFactor << std::endl;

  elevatorSparkMaxConfig.closedLoop
  .Pidf(kElevatorP, kElevatorI, kElevatorD, kElevatorFF)
  .OutputRange(kElevatorMinOutput, kElevatorMaxOutput);

  m_elevatorSparkMax.Configure(elevatorSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);  
  m_elevatorSparkMax.SetInverted(true); 

 // m_elevatorSparkMax.BurnFlash();

  std::cout << "Flash Burned on elevator subsystem\r\n";
  #else
  std::cout << "Flash was not burned on elevator subsystem\r\n";
  #endif

  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  
  auto nt_table = nt_inst.GetTable("Elevator");

  m_ElevatorLimitSwitch = nt_table->GetEntry("Elevator/Limit Switch");
  m_ElevatorDistance = nt_table->GetEntry("Elevator/Distance Extended");
 
    // Set the distance per pulse if needed
  m_elevatorEncoder.SetDistancePerPulse(0.02 / 360.0); // Example for a 360 PPR encoder
  
}

bool ElevatorSubsystem::ElevatorAtBase() {
  return !m_LimitSwitch.Get();
}

void ElevatorSubsystem::Periodic() {
  UpdateNTE();

  if (ElevatorAtBase())
   m_elevatorEncoder.Reset();
}

void ElevatorSubsystem::UpdateNTE() {
  m_ElevatorLimitSwitch.SetBoolean(ElevatorAtBase());
  m_ElevatorDistance.SetDouble(m_elevatorEncoder.GetDistance());
}

void ElevatorSubsystem::SetElevatorPower(double power) {
  /*if (power < 0.0 && m_elevatorEncoder.GetDistance() < -6.2) {
   m_elevatorSparkMax.Set(0.0);
  }
  else {
    if (ElevatorAtBase() && power > 0.0)
     m_elevatorSparkMax.Set(0.0);

    else {
     */m_elevatorSparkMax.Set(power);
    }
  //}}
  

