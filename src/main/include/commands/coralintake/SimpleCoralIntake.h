#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/MathUtil.h>

#include "subsystems/CoralIntakeSubsystem.h"

class SimpleCoralIntake
  : public frc2::CommandHelper<frc2::Command, SimpleCoralIntake> {
public:
  /**
   * Creates a new SimpleIntake.
   *
   * @param coralintake The pointer to the intake subsystem
   * @param opController The pointer to the operator controller
   */
  explicit SimpleCoralIntake(CoralIntakeSubsystem *coralintake);

  void Initialize() override;
  //void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  
private:
  CoralIntakeSubsystem* m_coralIntake;
};
