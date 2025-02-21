
#include "Constants.h"
#include "commands/elevator/StopElevator.h"

StopElevator::StopElevator(ElevatorSubsystem* elevator) : m_elevator{elevator} {
  AddRequirements(m_elevator);
}

void StopElevator::Initialize() {
#ifdef PRINTDEBUG 
  std::cout << "StopElevator Initialized\r\n";
#endif

  m_elevator->SetElevatorPower(0.0);
}

void StopElevator::End(bool interrupted) {
#ifdef PRINTDEBUG 
  std::cout << "StopElevator Ended\r\n";
#endif

  m_elevator->SetElevatorPower(0.0);
}