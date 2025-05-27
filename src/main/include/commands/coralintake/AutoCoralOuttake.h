
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/CoralIntakeSubsystem.h"

class AutoCoralOuttake
  : public frc2::CommandHelper<frc2::Command, AutoCoralOuttake> {
public:
  /**
   * Creates a new SimpleOuttake.
   *
   * @param coralintake The pointer to the intake subsystem
   * @param opController The pointer to the drive controller
   */
  explicit AutoCoralOuttake(CoralIntakeSubsystem* intake);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  CoralIntakeSubsystem* m_coralIntake;
};