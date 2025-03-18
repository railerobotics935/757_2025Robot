
#include "Constants.h"
#include "commands/elevator/ElevatorSetPointL1.h"

ElevatorSetPointL1::ElevatorSetPointL1(ElevatorSubsystem* elevator) : m_elevator{elevator} {
  AddRequirements(m_elevator);

  // Set the distance per pulse if needed
//  m_elevatorEncoder.SetDistancePerPulse(0.02 / 360.0); // Example for a 360 PPR encoder
}

void ElevatorSetPointL1::Execute() {
#ifdef PRINTDEBUG
  std::cout << "ElevatorSetPoint Initialized\r\n";
#endif

  m_elevator->GoToSetPoint(ElevatorConstants::kElevatorL1Position);
  
 
}

bool ElevatorSetPointL1::IsFinished(){
  if (abs(ElevatorConstants::kElevatorL1Position - m_elevator->GetElevatorPosition()) < 5.0)
    return true;
  else
    return false;

}
void ElevatorSetPointL1::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "ElevatorSetPoint Ended\r\n";
#endif

  m_elevator->SetElevatorPower(0.0);
}