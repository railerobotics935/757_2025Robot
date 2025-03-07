
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/ElevatorSubsystem.h"

class ElevatorSetPoint
  : public frc2::CommandHelper<frc2::Command, ElevatorSetPoint> {
public:
  /**
   * Creates a new ElevatorSetPoint.
   *
   * @param elevator The pointer to the intake subsystem
   */
  explicit ElevatorSetPoint(ElevatorSubsystem* elevator);

  void Execute() override;
  void End(bool interrupted) override;

private:
  ElevatorSubsystem* m_elevator;

  // Encoder motor controllers
  frc::Encoder m_elevatorEncoder{ElevatorConstants::kElevatorSensA, ElevatorConstants::kElevatorSensB};
};
