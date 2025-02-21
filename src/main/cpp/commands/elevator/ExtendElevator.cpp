
#include "Constants.h"
#include "commands/elevator/ExtendElevator.h"

ExtendElevator::ExtendElevator(ElevatorSubsystem* elevator) : m_elevator{elevator} {
  AddRequirements(m_elevator);
}

void ExtendElevator::Execute() {
#ifdef PRINTDEBUG
  std::cout << "ExtendElevator Initialized\r\n";
#endif

  m_elevator->SetElevatorPower(-1.0);
}

void ExtendElevator::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "ExtendElevator Ended\r\n";
#endif

  m_elevator->SetElevatorPower(0.0);
}