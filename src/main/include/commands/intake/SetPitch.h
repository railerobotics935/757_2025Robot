#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/MathUtil.h>

#include "subsystems/IntakeSubsystem.h"

class SetPitch
  : public frc2::CommandHelper<frc2::Command, SetPitch> {
public:
  /**
   * Creates a new SimpleIntake.
   *
   * @param intake The pointer to the intake subsystem
   * @param opController The pointer to the operator controller
   */
  explicit SetPitch(IntakeSubsystem* intake, frc::XboxController* operatorController);

  void Initialize() override;
  void Execute() override;
//  bool IsFinished() override;
  void End(bool interrupted) override;
  
private:
  IntakeSubsystem* m_intake;
  frc::XboxController* m_operatorController;
};
