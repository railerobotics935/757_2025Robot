// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <frc/DigitalInput.h>
#include <Constants.h>

class AlgaeIntakeSubsystem : public frc2::SubsystemBase {
 public:
  /**
   * Picks up game pieces :)
  */
  AlgaeIntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * @returns True if the upper algae limit switch is pressed
  */
  bool RightAlgaeIntakeRisen();
  bool LeftAlgaeIntakeRisen();


  // Sets the motor's power (between -1.0 and 1.0).
  void SetAlgaeIntakeMotorPower(double power);

  // Sets the angle of the pitch motor
  void SetPitchPosition(units::radian_t setAngle);
  
  // Sets the motor's power (between -1.0 and 1.0).
  void SetPitchPower(double power);

  /**
   * @return Direction pitch motor is moving
   */
  double GetDirection();

 private:

  void ConfigureAlgaeSparkMax();

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Motor Controllers
  rev::spark::SparkMax m_rightAlgaeIntakeSparkMax;
  rev::spark::SparkMax m_leftAlgaeIntakeSparkMax;
  
  rev::spark::SparkMax m_rightPitchSparkMax;
  rev::spark::SparkMax m_leftPitchSparkMax;

  // Encoders
  rev::spark::SparkAbsoluteEncoder m_rightPitchAbsoluteEncoder = m_rightPitchSparkMax.GetAbsoluteEncoder();
  rev::spark::SparkAbsoluteEncoder m_leftPitchAbsoluteEncoder = m_leftPitchSparkMax.GetAbsoluteEncoder();

  //PID for the pitch
  rev::spark::SparkClosedLoopController m_rightPitchPIDController = m_rightPitchSparkMax.GetClosedLoopController();
  rev::spark::SparkClosedLoopController m_leftPitchPIDController = m_leftPitchSparkMax.GetClosedLoopController();

  frc::DigitalInput m_rightAlgaeLimitSwitch{IntakeConstants::kRightAlgaeLimitSwitchPort};
  frc::DigitalInput m_leftAlgaeLimitSwitch{IntakeConstants::kLeftAlgaeLimitSwitchPort};


};
