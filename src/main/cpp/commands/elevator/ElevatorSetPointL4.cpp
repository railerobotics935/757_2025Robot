
#include "Constants.h"
#include "commands/elevator/ElevatorSetPointL4.h"

ElevatorSetPointL4::ElevatorSetPointL4(ElevatorSubsystem* elevator) : m_elevator{elevator} {
  AddRequirements(m_elevator);

  // Set the distance per pulse if needed
//  m_elevatorEncoder.SetDistancePerPulse(0.02 / 360.0); // Example for a 360 PPR encoder
}

void ElevatorSetPointL4::Execute() {
#ifdef PRINTDEBUG
  std::cout << "ElevatorSetPoint Initialized\r\n";
#endif

  m_elevator->GoToSetPoint(1.0);
 
}

void ElevatorSetPointL4::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "ElevatorSetPoint Ended\r\n";
#endif

  m_elevator->SetElevatorPower(0.0);
}