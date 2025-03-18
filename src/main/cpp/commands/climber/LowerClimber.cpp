
#include "Constants.h"
#include "commands/climber/LowerClimber.h"

LowerClimber::LowerClimber(ClimberSubsystem* climber) : m_climber{climber} {
  AddRequirements(m_climber);
}

void LowerClimber::Execute() {
#ifdef PRINTDEBUG
  std::cout << "LowerClimber Initialized\r\n";
#endif

  m_climber->SetClimberPower(1.0);
}


void LowerClimber::End(bool interrupted) {
  m_climber->SetClimberPower(0.0);
#ifdef PRINTDEBUG
  std::cout << "LowerClimber Ended\r\n";
#endif

}