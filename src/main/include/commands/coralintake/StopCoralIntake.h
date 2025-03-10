
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/CoralIntakeSubsystem.h"

class StopCoralIntake
  : public frc2::CommandHelper<frc2::Command, StopCoralIntake> {
public:
  /**
   * Creates a new StopIntake.
   *
   * @param intake The pointer to the intake subsystem
   */
  explicit StopCoralIntake(CoralIntakeSubsystem* intake);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  CoralIntakeSubsystem* m_coralIntake;
};
