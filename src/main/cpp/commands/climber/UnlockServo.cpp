
#include "Constants.h"
#include "commands/climber/UnlockServo.h"

UnlockServo::UnlockServo(ClimberSubsystem* climber) : m_climber{climber} {
  AddRequirements(m_climber);
}

void UnlockServo::Execute() {
#ifdef PRINTDEBUG
  std::cout << "UnlockServo Initialized\r\n";
#endif

  m_climber->LatchServo(ClimberConstants::kServoUnlockedPosition);
}

void UnlockServo::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "UnlockServo Ended\r\n";
#endif
}