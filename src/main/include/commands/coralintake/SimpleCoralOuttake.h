
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/IntakeSubsystem.h"

class SimpleOuttake
  : public frc2::CommandHelper<frc2::Command, SimpleOuttake> {
public:
  /**
   * Creates a new SimpleOuttake.
   *
   * @param intake The pointer to the intake subsystem
   * @param opController The pointer to the drive controller
   */
  explicit SimpleOuttake(IntakeSubsystem* intake);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  IntakeSubsystem* m_intake;
};