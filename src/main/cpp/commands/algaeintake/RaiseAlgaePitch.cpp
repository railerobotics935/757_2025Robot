
#include "Constants.h"
#include "commands/algaeintake/RaiseAlgaePitch.h"

RaiseAlgaePitch::RaiseAlgaePitch(AlgaeIntakeSubsystem *algaeintake) : m_algaeIntake{algaeintake} {

  AddRequirements(m_algaeIntake);
}

void RaiseAlgaePitch::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_algaeIntake->SetPitchPower(0.4);
}


void RaiseAlgaePitch::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
  m_algaeIntake->SetPitchPower(0.0);
}