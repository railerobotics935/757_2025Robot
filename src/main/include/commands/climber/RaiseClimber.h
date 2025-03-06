
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/ClimberSubsystem.h"

class RaiseClimber
  : public frc2::CommandHelper<frc2::Command, RaiseClimber> {
public:
  /**
   * Creates a new RaiseClimber.
   *
   * @param climber The pointer to the intake subsystem
   */
  explicit RaiseClimber(ClimberSubsystem* climber);

  void Execute() override;
  void End(bool interrupted) override;
  
private:
  ClimberSubsystem* m_climber;
};
