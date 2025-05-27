
#include "commands/coralintake/AutoCoralOuttake.h"


AutoCoralOuttake::AutoCoralOuttake(CoralIntakeSubsystem* coralintake) : m_coralIntake{coralintake} {
  // Initilize local copys of pointers
  m_coralIntake = coralintake;

  // Add reqierments for the command
  AddRequirements(m_coralIntake);
}

void AutoCoralOuttake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleOuttake Initialized\r\n";
#endif
  m_coralIntake->SetCoralIntakeMotorPower(0.2);
  
}

void AutoCoralOuttake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleOuttake Ended\r\n";
#endif
  m_coralIntake->SetCoralIntakeMotorPower(0.0);
  //m_coralIntake->SetCoralIntakeAngle(0.0);

}