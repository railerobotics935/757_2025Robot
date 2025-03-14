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
   */
  explicit SimpleAlgaeIntake(AlgaeIntakeSubsystem *algaeintake);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  AlgaeIntakeSubsystem* m_algaeIntake;
  frc::XboxController* m_operatorController;
};
