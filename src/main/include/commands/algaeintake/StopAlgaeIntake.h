
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/AlgaeIntakeSubsystem.h"

class StopAlgaeIntake
  : public frc2::CommandHelper<frc2::Command, StopAlgaeIntake> {
public:
  /**
   * Creates a new StopIntake.
   *
   * @param algaeintake The pointer to the intake subsystem
   */
  explicit StopAlgaeIntake(AlgaeIntakeSubsystem* algaeintake);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  AlgaeIntakeSubsystem* m_algaeIntake;
};
