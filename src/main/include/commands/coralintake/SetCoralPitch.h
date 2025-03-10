#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/MathUtil.h>

#include "subsystems/CoralIntakeSubsystem.h"

class SetCoralPitch
  : public frc2::CommandHelper<frc2::Command, SetCoralPitch> {
public:
  /**
   * Creates a new SimpleIntake.
   *
   * @param intake The pointer to the intake subsystem
   * @param opController The pointer to the operator controller
   */
  explicit SetCoralPitch(CoralIntakeSubsystem* intake, frc::XboxController* operatorController);

  void Initialize() override;
  void Execute() override;
//  bool IsFinished() override;
  void End(bool interrupted) override;
  
private:
  CoralIntakeSubsystem* m_coralIntake;
  frc::XboxController* m_operatorController;
};
