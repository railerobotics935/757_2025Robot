#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/MathUtil.h>

#include "subsystems/AlgaeIntakeSubsystem.h"

class SimpleAlgaeIntake
  : public frc2::CommandHelper<frc2::Command, SimpleAlgaeIntake> {
public:
  /**
   * Creates a new SimpleIntake.
   *
   * @param algaeintake The pointer to the intake subsystem
   * @param opController The pointer to the operator controller
   */
  explicit SimpleAlgaeIntake(AlgaeIntakeSubsystem* intake, frc::XboxController* operatorController);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  
private:
  AlgaeIntakeSubsystem* m_algaeIntake;
  frc::XboxController* m_operatorController;
};
