
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/ElevatorSubsystem.h"

class ElevatorSetPointL4
  : public frc2::CommandHelper<frc2::Command, ElevatorSetPointL4> {
public:
  /**
   * Creates a new ElevatorSetPoint.
   *
   * @param elevator The pointer to the intake subsystem
   */
  explicit ElevatorSetPointL4(ElevatorSubsystem* elevator);

  void Execute() override;
  void End(bool interrupted) override;

private:
  ElevatorSubsystem* m_elevator;

  // Encoder motor controllers
};
