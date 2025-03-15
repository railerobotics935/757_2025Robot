
#include "Constants.h"
#include "commands/algaeintake/LowerAlgaePitch.h"

LowerAlgaePitch::LowerAlgaePitch(AlgaeIntakeSubsystem *algaeintake) : m_algaeIntake{algaeintake} {

  AddRequirements(m_algaeIntake);
}

void LowerAlgaePitch::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_algaeIntake->SetPitchPower(-0.4);
}


void LowerAlgaePitch::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
  m_algaeIntake->SetPitchPower(0.0);
}