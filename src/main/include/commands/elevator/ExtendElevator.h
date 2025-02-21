
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/ElevatorSubsystem.h"

class ExtendElevator
  : public frc2::CommandHelper<frc2::Command, ExtendElevator> {
public:
  /**
   * Creates a new ExtendElevator.
   *
   * @param elevator The pointer to the intake subsystem
   */
  explicit ExtendElevator(ElevatorSubsystem* elevator);

  void Execute() override;
  void End(bool interrupted) override;
  
private:
  ElevatorSubsystem* m_elevator;
};
