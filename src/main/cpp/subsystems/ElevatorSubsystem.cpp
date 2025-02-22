// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"
#include "Constants.h"
#include <networktables/NetworkTableInstance.h>

using namespace ElevatorConstants;

ElevatorSubsystem::ElevatorSubsystem() {

  // Burn flash only if desired - true set in constants
  #ifdef BURNCLIMBERSPARKMAX
  // Restore deafults
 m_elevatorMotor.RestoreFactoryDefaults();
  m_rightElevatorMotor.RestoreFactoryDefaults();
  
  // Set converstion factors for encoders
 m_elevatorEncoder.SetPositionConversionFactor(kPositionFactor);
 m_elevatorEncoder.SetVelocityConversionFactor(kVelocityFactor);

  m_rightElevatorEncoder.SetPositionConversionFactor(kPositionFactor);
  m_rightElevatorEncoder.SetVelocityConversionFactor(kVelocityFactor);
  
  // Set Idle mode (what to do when not commanded at a speed)
 m_elevatorMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  m_rightElevatorMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
 m_elevatorMotor.SetSmartCurrentLimit(kMotorCurrentLimit.value());
  m_rightElevatorMotor.SetSmartCurrentLimit(kMotorCurrentLimit.value());
  
  // Invert the left becasue its mirrored
 m_elevatorMotor.SetInverted(true);

 m_elevatorMotor.BurnFlash();
  m_rightElevatorMotor.BurnFlash();

  std::cout << "Flash Burned on elevator subsystem\r\n";
  #else
  std::cout << "Flash was not burned on elevator subsystem\r\n";
  #endif

  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  
  auto nt_table = nt_inst.GetTable("Elevator");

  m_baseElevatorLimitSwitch = nt_table->GetEntry("Left Elevator/Limit Switch");
  m_ElevatorDistance = nt_table->GetEntry("Left Elevator/Distance Extended");
  m_upperElevatorLimitSwitch = nt_table->GetEntry("Right Elevator/Limit Switch");
}

bool ElevatorSubsystem::ElevatorAtBase() {
  return m_baseLimitSwitch.Get();
}

bool ElevatorSubsystem::ElevatorRisen() {
  return m_upperLimitSwitch.Get();
}

void ElevatorSubsystem::Periodic() {
  UpdateNTE();

  if (ElevatorAtBase())
   m_elevatorEncoder.SetPosition(0.0);
//  if (ElevatorRisen())
//    m_rightElevatorEncoder.SetPosition(0.0);
}

void ElevatorSubsystem::UpdateNTE() {
  m_baseElevatorLimitSwitch.SetBoolean(ElevatorAtBase());
  m_ElevatorDistance.SetDouble(m_elevatorEncoder.GetPosition());
  m_upperElevatorLimitSwitch.SetBoolean(ElevatorRisen());
}

void ElevatorSubsystem::SetElevatorPower(double power) {
 /* if (power < 0.0 &&m_elevatorEncoder.GetPosition() < -6.2) {
   m_elevatorMotor.Set(0.0);
  }
  else {
    if (ElevatorAtBase() && power > 0.0)*/
     //m_elevatorMotor.Set(0.0);
    //else
     m_elevatorMotor.Set(power);
  }

//  if (power < 0.0 && m_rightElevatorEncoder.GetPosition() < -6.2) {
//    m_rightElevatorMotor.Set(0.0);
//  }
//  else {
//    if (ElevatorRisen() && power > 0.0)
//      m_rightElevatorMotor.Set(0.0);
//    else
//      m_rightElevatorMotor.Set(power);
//  }
  
  
  
//}

