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

#include "Constants.h"

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  /**
   * Creates a Elevator subsystem.
   * Currently for both indiviual elevators (two phystical subsystems)
   * but coding it as one
  */
  ElevatorSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override; 

  /**
   * @returns True if the base elevator limit switch is pressed
  */
  bool ElevatorAtBase();

  /**
   * @returns True if the upper elevator limit switch is pressed
  */
  bool ElevatorRisen();

  /**
   * Set the elevator moter to a power
   * 
   * @param power Power to set the motor power
  */
  void SetElevatorPower(double power);

  /**
   * Set the elevator motor power invidualy
  */
  void SetIndividualElevatorPower(double power);

  /**
   * Updates NetworkTableEntries
  */
  void UpdateNTE();

 private:

  nt::NetworkTableEntry m_baseElevatorLimitSwitch;
  nt::NetworkTableEntry m_ElevatorDistance;
  nt::NetworkTableEntry m_upperElevatorLimitSwitch;


  // Motor Controllers
  rev::spark::SparkMax m_elevatorMotor{ElevatorConstants::kID, ElevatorConstants::kMotorType};
// rev::spark::SparkMax m_rightElevatorMotor{ElevatorConstants::RightElevator::kID, ElevatorConstants::kMotorType};
  
  // Encoders motor controllers
  rev::spark::SparkRelativeEncoder m_elevatorEncoder = m_elevatorMotor.GetEncoder();
  // rev::spark::SparkRelativeEncoder m_rightElevatorEncoder = m_rightElevatorMotor.GetEncoder();

  // Limit switch is a digital input in the DIO port (digital input output)
  frc::DigitalInput m_baseLimitSwitch{ElevatorConstants::kBaseLimitSwitchPort};
  frc::DigitalInput m_upperLimitSwitch{ElevatorConstants::kUpperLimitSwitchPort};
};