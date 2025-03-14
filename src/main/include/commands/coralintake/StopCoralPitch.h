#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/MathUtil.h>

#include "subsystems/CoralIntakeSubsystem.h"

class StopCoralPitch
  : public frc2::CommandHelper<frc2::Command, StopCoralPitch> {
public:
  /**
   * Creates a new SimpleIntake.
   *
   * @param coralintake The pointer to the intake subsystem
   * @param opController The pointer to the operator controller
   */
  explicit StopCoralPitch(CoralIntakeSubsystem *coralintake);

  void Initialize() override;
  //void Execute() override;
  void End(bool interrupted) override;
  
private:
  CoralIntakeSubsystem* m_coralIntake;
};
