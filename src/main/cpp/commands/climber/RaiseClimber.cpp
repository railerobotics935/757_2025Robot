
#include "Constants.h"
#include "commands/climber/RaiseClimber.h"

RaiseClimber::RaiseClimber(ClimberSubsystem* climber) : m_climber{climber} {
  AddRequirements(m_climber);
}

void RaiseClimber::Execute() {
#ifdef PRINTDEBUG
  std::cout << "RaiseClimber Initialized\r\n";
#endif

  m_climber->SetClimberPower(-1.0);
}

void RaiseClimber::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "RaiseClimber Ended\r\n";
#endif
}