
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "subsystems/ShooterSubsystem.h"
#include "subsystems/GuftSubsystem.h"

/**
 * Command that shoots the note with the robot up to the amp
 */
class AmpShoot 
  : public frc2::CommandHelper<frc2::Command, AmpShoot> {
public:
  /**
   * Sets shooter to values in networktable entries
   * 
   * @param shooter memory adress of shooter subsystem
   * @param guft memory adress of the guft subsystem
  */
  explicit AmpShoot(ShooterSubsystem* shooter, GuftSubsystem* guft);

  void Initialize() override;
  void End(bool interrupted) override;

private:
// Declare local subsystems
ShooterSubsystem* m_shooter;
GuftSubsystem* m_guft;
};