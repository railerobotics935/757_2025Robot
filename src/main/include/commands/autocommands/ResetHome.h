#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/CoralPitchSubsystem.h"

class ResetHome
  : public frc2::CommandHelper<frc2::Command, ResetHome> {
public:
  /**
   * Creates a new Reset home command.
   *
   * @param elevator The pointer to the elevator subsystem
   * @param coarlPitch The pointer to the coral pitch subsystem
   */
  explicit ResetHome(ElevatorSubsystem* elevator, CoralPitchSubsystem* coralPitch);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  ElevatorSubsystem* m_elevator;
  CoralPitchSubsystem* m_coralPitch;
};
