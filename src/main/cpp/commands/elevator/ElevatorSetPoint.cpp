
#include "Constants.h"
#include "commands/elevator/ElevatorSetPoint.h"

ElevatorSetPoint::ElevatorSetPoint(ElevatorSubsystem* elevator) : m_elevator{elevator} {
  AddRequirements(m_elevator);

  // Set the distance per pulse if needed
  m_elevatorEncoder.SetDistancePerPulse(0.02 / 360.0); // Example for a 360 PPR encoder
}

void ElevatorSetPoint::Execute() {
#ifdef PRINTDEBUG
  std::cout << "ElevatorSetPoint Initialized\r\n";
#endif


  if (m_elevatorEncoder.GetDistance() < 6) {
    m_elevator->SetElevatorPower(1.0);
  }
  else if (m_elevatorEncoder.GetDistance() > 6) {
    m_elevator->SetElevatorPower(-0.02);
  }
  else {
    m_elevator->SetElevatorPower(0.02);
  }
}

void ElevatorSetPoint::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "ElevatorSetPoint Ended\r\n";
#endif

  m_elevator->SetElevatorPower(0.0);
}