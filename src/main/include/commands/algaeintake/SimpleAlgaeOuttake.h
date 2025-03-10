
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/AlgaeIntakeSubsystem.h"

class SimpleAlgaeOuttake
  : public frc2::CommandHelper<frc2::Command, SimpleAlgaeOuttake> {
public:
  /**
   * Creates a new SimpleOuttake.
   *
   * @param intake The pointer to the intake subsystem
   * @param opController The pointer to the drive controller
   */
  explicit SimpleAlgaeOuttake(AlgaeIntakeSubsystem* intake);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  AlgaeIntakeSubsystem* m_algaeIntake;
};