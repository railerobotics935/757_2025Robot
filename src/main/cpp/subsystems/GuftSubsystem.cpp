// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GuftSubsystem.h"
#include <networktables/NetworkTableInstance.h>
#include <rev/config/SparkMaxConfig.h>

using namespace GuftConstants;

GuftSubsystem::GuftSubsystem(double guftAngleOffset) : m_guftMotor{kGuftID, kGuftMotorType} {
  
  // Burn flash only if desired - true set in constants
  #ifdef BURNGUFTSPARKMAX 
  // Restore deafults
  rev::spark::SparkMaxConfig config{};
   // Enable Voltage
  config
  .VoltageCompensation(RobotConstants::kVoltageCompentationValue)
    // Set Idle mode (what to do when not commanded at a speed)
  .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake)
  .SmartCurrentLimit(kGuftMotorCurrentLimit.value())
  .Inverted(true);
 
  // Set converstion factors for encoders
  config.absoluteEncoder
    .PositionConversionFactor(kGuftPositionFactor)
    .VelocityConversionFactor(kGuftEncoderVelocityFactor)
    .ZeroOffset(guftAngleOffset);
  
  // Set PID Constants
  config.closedLoop
  .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
  .Pidf(kGuftP, kGuftI, kGuftD,kGuftFF)
  .OutputRange(kGuftMin, kGuftMax);
  

  // Set the PID Controller to use the duty cycle encoder on the swerve
  // module instead of the built in NEO550 encoder.


  m_guftMotor.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
  std::cout << "Flash Burned on Guft subsystem\r\n";
  #else
  std::cout << "Flash was not burned on Guft subsystem\r\n";
  #endif

  m_guftAngleOffset = guftAngleOffset;

  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("Guft");  

  nte_guftAngle = nt_table->GetEntry("Guft Angle");
  nte_guftSetpoint = nt_table->GetEntry("Guft Setpoint");

}

void GuftSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  nte_guftAngle.SetDouble((double)m_guftAbsoluteEncoder.GetPosition());
}

double GuftSubsystem::GetGuftAngle() {
  return m_guftAbsoluteEncoder.GetPosition();
}

void GuftSubsystem::SetGuftAngle(units::radian_t angle) {
  // Set the setpoint as the input angle
  if ((double)angle > kMaxGuftAngle)
    angle = (units::radian_t)kMaxGuftAngle;
  if ((double)angle < kMinGuftAngle)
    angle = (units::radian_t)kMinGuftAngle;
  m_guftPIDController.SetReference((double)angle, rev::spark::SparkMax::ControlType::kPosition);
  nte_guftSetpoint.SetDouble((double)angle);
}

bool GuftSubsystem::AtAngleSetpoint() {
  if (abs(nte_guftSetpoint.GetDouble(1.0) - m_guftAbsoluteEncoder.GetPosition()) < 0.05) // in radians
    return true;
  else
    return false;
}