#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/MathUtil.h>

#include "subsystems/CoralPitchSubsystem.h"

class SetWristToL1
  : public frc2::CommandHelper<frc2::Command, SetWristToL1> {
public:
  /**
   * Creates a new command for setting wrist to L1.
   *
   * @param coralpitch The pointer to the pitch subsystem
   */
  explicit SetWristToL1(CoralPitchSubsystem *coralpitch);

  void Initialize() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  
private:
  CoralPitchSubsystem* m_coralPitch;
};
