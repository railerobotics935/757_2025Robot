
#include "Constants.h"
#include "commands/coralintake/SetCoralPitch.h"

SetCoralPitch::SetCoralPitch(CoralIntakeSubsystem* coralintake) : m_coralIntake{coralintake} {

  AddRequirements(m_coralIntake);
}

void SetCoralPitch::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SetPitch Initialized\r\n";
#endif
  m_coralIntake->SetCoralPitchPower(1.0);
}

void SetCoralPitch::Execute() {
  m_coralIntake->SetCoralPitchPosition(units::radian_t(std::numbers::pi / 4));
}

void SetCoralPitch::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SetPitch Ended\r\n";
#endif
  m_coralIntake->SetCoralPitchPower(0.0);
}