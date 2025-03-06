
#include "Constants.h"
#include "commands/climber/LockServo.h"

LockServo::LockServo(ClimberSubsystem* climber) : m_climber{climber} {
  AddRequirements(m_climber);
}

void LockServo::Execute() {
#ifdef PRINTDEBUG
  std::cout << "LockServo Initialized\r\n";
#endif

  m_climber->LatchServo(ClimberConstants::kServoLockedPosition);
}

void LockServo::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "LockServo Ended\r\n";
#endif
}