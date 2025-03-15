
#include "Constants.h"
#include "commands/algaeintake/StopAlgaePitch.h"

StopAlgaePitch::StopAlgaePitch(AlgaeIntakeSubsystem *algaeintake) : m_algaeIntake{algaeintake} {

  AddRequirements(m_algaeIntake);
}

void StopAlgaePitch::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_algaeIntake->SetPitchPower(0.0);
}


void StopAlgaePitch::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
  m_algaeIntake->SetPitchPower(0.0);
}