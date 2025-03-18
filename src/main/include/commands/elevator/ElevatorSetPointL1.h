
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/ElevatorSubsystem.h"

class ElevatorSetPointL1
  : public frc2::CommandHelper<frc2::Command, ElevatorSetPointL1> {
public:
  /**
   * Creates a new ElevatorSetPoint.
   *
   * @param elevator The pointer to the intake subsystem
   */
  explicit ElevatorSetPointL1(ElevatorSubsystem* elevator);

  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

private:
  ElevatorSubsystem* m_elevator;

  // Encoder motor controllers
};
