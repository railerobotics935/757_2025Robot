#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/MathUtil.h>

#include "subsystems/CoralPitchSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"

class GoToIntake
  : public frc2::CommandHelper<frc2::Command, GoToIntake> {
public:
  /**
   * Creates a new command for setting wrist to L1.
   *
   * @param coralpitch The pointer to the pitch subsystem
   */
  explicit GoToIntake(CoralPitchSubsystem *coralpitch, ElevatorSubsystem *elevator);

  void Initialize() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  
private:
  CoralPitchSubsystem* m_coralPitch;
  ElevatorSubsystem* m_elevator;
};
