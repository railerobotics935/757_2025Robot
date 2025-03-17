// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include "pathplanner/lib/commands/PathPlannerAuto.h"
#include "pathplanner/lib/auto/NamedCommands.h"

#include "subsystems/DriveSubsystem.h"
#include "Constants.h"

using namespace DriveConstants;
using namespace pathplanner;

/**
 * Idea:
 * 
 * m_drive.SetDefaultCommand(std::move(m_driveCommand));
 *   
 * it says it works, but it hasen't been tested yet. I don't know how different it
 * is, if it is better or not
*/

RobotContainer::RobotContainer() {
  m_revPDH.SetSwitchableChannel(true); //-------------------------------------------------------------------------------------

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(std::move(m_driveWithController));
  m_coralIntake.SetDefaultCommand(std::move(m_stopCoralIntake));
  m_coralPitch.SetDefaultCommand(std::move(m_stopCoralPitch));
  m_elevator.SetDefaultCommand(std::move(m_stopElevator));
  m_climber.SetDefaultCommand(std::move(m_stopClimber));
  m_algaeIntake.SetDefaultCommand(std::move(m_stopAlgaeIntake));
  m_algaeIntake.SetDefaultCommand(std::move(m_stopAlgaePitch));
  
  frc::Shuffleboard::GetTab("Autonomous").Add(m_autoChooser);

  m_autoChooser.SetDefaultOption("New Auto", m_newAuto);
}

void RobotContainer::ConfigureButtonBindings() {

  // Create new button bindings
  frc2::JoystickButton resetButton(&m_driveController, ControllerConstants::kResetGyroButtonIndex); 
  frc2::JoystickButton robotRelativeButton(&m_driveController, ControllerConstants::kRobotRelativeButtonIndex);
  frc2::JoystickButton fieldRelativeButton(&m_driveController, ControllerConstants::kFieldRelativeButtonIndex);
  frc2::JoystickButton raiseClimberButton(&m_driveController, ControllerConstants::kRaiseClimberButton);
  frc2::JoystickButton lowerClimberButton(&m_driveController, ControllerConstants::kLowerClimberButton);

  frc2::JoystickButton coralIntakeButton(&m_operatorController, ControllerConstants::kCoralIntakeButton);
  frc2::JoystickButton coralOuttakeButton(&m_operatorController, ControllerConstants::kCoralOuttakeButton);
  frc2::JoystickButton algaeIntakeButton(&m_operatorController, ControllerConstants::kAlgaeIntakeButton);
  frc2::JoystickButton algaeOuttakeButton(&m_operatorController, ControllerConstants::kAlgaeOuttakeButton);
  frc2::JoystickButton raiseAlgaePitchButton(&m_operatorController, ControllerConstants::kAlgaePitchRaiseButton);
  frc2::JoystickButton lowerAlgaePitchButton(&m_operatorController, ControllerConstants::kAlgaePitchLowerButton);
  frc2::JoystickButton raiseElevatorButton(&m_operatorController, ControllerConstants::kExtendElevatorButton);
  frc2::JoystickButton lowerElevatorButton(&m_operatorController, ControllerConstants::kRetractElevatorButton);
  frc2::JoystickButton setPointOneButton(&m_operatorController, ControllerConstants::kElevatorSetPointButton);
  frc2::JoystickButton lowerCoralPitchButton(&m_operatorController, ControllerConstants::kCoralPitchLowerButton);
  frc2::JoystickButton raiseCoralPitchButton(&m_operatorController, ControllerConstants::kCoralPitchRaiseButton);


  // Bind commands to button triggers
  resetButton.OnTrue(frc2::cmd::RunOnce([&] {m_drive.ZeroHeading();}, {}));
  robotRelativeButton.OnTrue(frc2::cmd::RunOnce([&] {m_drive.SetRobotRelative();}, {}));
  fieldRelativeButton.OnTrue(frc2::cmd::RunOnce([&] {m_drive.SetFieldRelative();}, {}));
  coralIntakeButton.WhileTrue(SimpleCoralIntake{&m_coralIntake}.ToPtr());
  coralOuttakeButton.WhileTrue(SimpleCoralOuttake{&m_coralIntake}.ToPtr());
  raiseCoralPitchButton.WhileTrue(RaiseCoralPitch{&m_coralPitch}.ToPtr());
  lowerCoralPitchButton.WhileTrue(LowerCoralPitch{&m_coralPitch}.ToPtr());
  algaeIntakeButton.WhileTrue(SimpleAlgaeIntake{&m_algaeIntake}.ToPtr());
  algaeOuttakeButton.WhileTrue(SimpleAlgaeOuttake{&m_algaeIntake}.ToPtr());
  raiseAlgaePitchButton.WhileTrue(RaiseAlgaePitch{&m_algaeIntake}.ToPtr());
  lowerAlgaePitchButton.WhileTrue(LowerAlgaePitch{&m_algaeIntake}.ToPtr());
  raiseElevatorButton.WhileTrue(ExtendElevator{&m_elevator}.ToPtr());
  lowerElevatorButton.WhileTrue(RetractElevator{&m_elevator}.ToPtr());
  raiseClimberButton.WhileTrue(RaiseClimber{&m_climber}.ToPtr());
  lowerClimberButton.WhileTrue(LowerClimber{&m_climber}.ToPtr());
  setPointOneButton.OnTrue(ElevatorSetPoint{&m_elevator}.ToPtr());

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // Builds and returns auto commands from pathplanner
  return PathPlannerAuto(m_autoChooser.GetSelected()).ToPtr();
}
