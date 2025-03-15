#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/MathUtil.h>

#include "subsystems/AlgaeIntakeSubsystem.h"

class StopAlgaePitch
  : public frc2::CommandHelper<frc2::Command, StopAlgaePitch> {
public:
  /**
   * Creates a new SimpleIntake.
   *
   * @param algaeintake The pointer to the intake subsystem
   */
  explicit StopAlgaePitch(AlgaeIntakeSubsystem *algaeintake);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  AlgaeIntakeSubsystem* m_algaeIntake;
  frc::XboxController* m_operatorController;
};
