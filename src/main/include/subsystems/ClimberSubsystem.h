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
#include <frc/Servo.h>

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
   * @returns True if the base climber limit switch is pressed
  */
  bool ClimberAtBase();

  /**
   * @returns True if the upper climber limit switch is pressed
  */
  bool ClimberRisen();

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

  /**
   * Locks the servo latch so the robot doesn't fall after being powered off
   */
  void LatchServo(double value);

 private:

  nt::NetworkTableEntry m_ClimberLimitSwitch;
  nt::NetworkTableEntry m_ClimberDistance;

  // Motor Controllers
  rev::spark::SparkMax m_climberSparkMax{ClimberConstants::kMotorID, ClimberConstants::kMotorType};
  
  // Encoders motor controllers
  rev::spark::SparkRelativeEncoder m_climberEncoder = m_climberSparkMax.GetEncoder();

  // Limit switch is a digital input in the DIO port (digital input output)
  frc::DigitalInput m_LimitSwitch{ClimberConstants::kLimitSwitchPort};

  frc::Servo m_climberServo{ClimberConstants::kServoPort};
};