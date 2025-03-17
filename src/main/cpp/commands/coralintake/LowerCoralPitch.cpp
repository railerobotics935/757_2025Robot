
#include "Constants.h"
#include "commands/coralintake/LowerCoralPitch.h"

LowerCoralPitch::LowerCoralPitch(CoralPitchSubsystem *coralpitch) : m_coralPitch{coralpitch} {

  m_coralPitch = coralpitch;

  AddRequirements(m_coralPitch);
}

void LowerCoralPitch::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  currentCoralAngle = m_coralPitch->GetCoralIntakeAngle();
  //m_coralIntake->SetCoralIntakeAngle(0.25);
}

void LowerCoralPitch::Execute() {
  currentCoralAngle += 0.004;
  m_coralPitch->SetCoralIntakeAngle(currentCoralAngle);

}

void LowerCoralPitch::End(bool interrupted) {
  //m_coralIntake->SetCoralIntakeAngle(0.0);
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}