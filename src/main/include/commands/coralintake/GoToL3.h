#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/MathUtil.h>

#include "subsystems/CoralPitchSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"

class GoToL3
  : public frc2::CommandHelper<frc2::Command, GoToL3> {
public:
  /**
   * Creates a new command for setting wrist to L1.
   *
   * @param coralpitch The pointer to the pitch subsystem
   */
  explicit GoToL3(CoralPitchSubsystem *coralpitch, ElevatorSubsystem *elevator);

  void Initialize() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  
private:
  CoralPitchSubsystem* m_coralPitch;
  ElevatorSubsystem* m_elevator;
};
