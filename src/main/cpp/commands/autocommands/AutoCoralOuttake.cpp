
#include "commands/autocommands/AutoCoralOuttake.h"
#include <frc/Timer.h>


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
  m_timer.Start();
  m_coralIntake->SetCoralIntakeMotorPower(0.5);
}

bool AutoCoralOuttake::IsFinished(){
    if (m_timer.Get() > units::time::second_t(1.0))
        return true;
    else
        return false;
}

void AutoCoralOuttake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleOuttake Ended\r\n";
#endif
  m_coralIntake->SetCoralIntakeMotorPower(0.0);
  //m_coralIntake->SetCoralIntakeAngle(0.0);

}