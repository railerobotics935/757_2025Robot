
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/ElevatorSubsystem.h"

class StopElevator
  : public frc2::CommandHelper<frc2::Command, StopElevator> {
public:
  /**
   * Creates a new StopElevator.
   *
   * @param elevator The pointer to the intake subsystem
   */
  explicit StopElevator(ElevatorSubsystem* elevator);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  ElevatorSubsystem* m_elevator;
};
