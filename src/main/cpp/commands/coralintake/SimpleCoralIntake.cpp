
#include "Constants.h"
#include "commands/coralintake/SimpleCoralIntake.h"

SimpleCoralIntake::SimpleCoralIntake(CoralIntakeSubsystem *coralintake) : m_coralIntake{coralintake} {

  m_coralIntake = coralintake;

  AddRequirements(m_coralIntake);
}

void SimpleCoralIntake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_coralIntake->SetCoralIntakeMotorPower(-0.5);
  //m_coralIntake->SetCoralIntakeAngle(0.3);
}


bool SimpleCoralIntake::IsFinished() {
  if(m_coralIntake->CoralInIntake() && m_coralIntake->GetDirection() < 0) {
    return true;
  }
  else {
    return false;
  }
}


void SimpleCoralIntake::End(bool interrupted) {
  m_coralIntake->SetCoralIntakeMotorPower(0.0);
  //m_coralIntake->SetCoralIntakeAngle(0.0);
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}