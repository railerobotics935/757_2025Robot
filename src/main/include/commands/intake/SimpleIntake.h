#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/MathUtil.h>

#include "subsystems/IntakeSubsystem.h"

class SimpleIntake
  : public frc2::CommandHelper<frc2::Command, SimpleIntake> {
public:
  /**
   * Creates a new SimpleIntake.
   *
   * @param intake The pointer to the intake subsystem
   * @param opController The pointer to the operator controller
   */
  explicit SimpleIntake(IntakeSubsystem* intake, frc::XboxController* operatorController);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  
private:
  IntakeSubsystem* m_intake;
  frc::XboxController* m_operatorController;
};
