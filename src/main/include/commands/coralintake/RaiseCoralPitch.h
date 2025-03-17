#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/MathUtil.h>

#include "subsystems/CoralPitchSubsystem.h"

class RaiseCoralPitch
  : public frc2::CommandHelper<frc2::Command, RaiseCoralPitch> {
public:
  /**
   * Creates a new SimpleIntake.
   *
   * @param coralpitch The pointer to the pitch subsystem
   * @param opController The pointer to the operator controller
   */
  explicit RaiseCoralPitch(CoralPitchSubsystem *coralpitch);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  
private:
  CoralPitchSubsystem* m_coralPitch;
  double currentCoralAngle;
};
