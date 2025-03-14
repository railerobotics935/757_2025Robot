// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <units/length.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/DigitalInput.h>
#include <frc/SensorUtil.h>
#include <frc/Encoder.h>

#include "Constants.h"

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  /**
   * Creates a Climber subsystem.
   * Currently for both indiviual climbers (two phystical subsystems)
   * but coding it as one
  */
  ClimberSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override; 

  /**
   * Set the climber motor to a power
   * 
   * @param power Power to set the motor power
  */
  void SetClimberPower(double power);

  /**
   * Updates NetworkTableEntries
  */
  void UpdateNTE();

 private:

  //nt::NetworkTableEntry m_ClimberLimitSwitch;
  nt::NetworkTableEntry m_ClimberDistance;

  // Motor Controllers
  rev::spark::SparkMax m_climberSparkMax{ClimberConstants::kClimberMotorID, ClimberConstants::kMotorType};
  
  // Encoders motor controllers
  rev::spark::SparkRelativeEncoder m_climberEncoder = m_climberSparkMax.GetEncoder();

};