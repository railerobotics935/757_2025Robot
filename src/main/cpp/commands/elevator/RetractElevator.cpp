
#include "Constants.h"
#include "commands/elevator/RetractElevator.h"

RetractElevator::RetractElevator(ElevatorSubsystem* elevator) : m_elevator{elevator} {
  AddRequirements(m_elevator);
}

void RetractElevator::Execute() {
#ifdef PRINTDEBUG
  std::cout << "RetractElevator Initialized\r\n";
#endif

  m_elevator->SetElevatorPower(-0.5);
}

void RetractElevator::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "RetractElevator Ended\r\n";
#endif  

  m_elevator->SetElevatorPower(0.0);
}