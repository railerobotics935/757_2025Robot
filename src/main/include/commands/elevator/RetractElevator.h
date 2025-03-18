
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/ElevatorSubsystem.h"

class RetractElevator
  : public frc2::CommandHelper<frc2::Command, RetractElevator> {
public:
  /**
   * Creates a new RetractElevator.
   *
   * @param elevator The pointer to the intake subsystem
   */
  explicit RetractElevator(ElevatorSubsystem* elevator);

  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  
private:
  ElevatorSubsystem* m_elevator;
};
