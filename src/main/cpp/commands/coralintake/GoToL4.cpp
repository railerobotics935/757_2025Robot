#include "Constants.h"
#include "commands/coralintake/GoToL4.h"

GoToL4::GoToL4(CoralPitchSubsystem *coralpitch, ElevatorSubsystem *elevator) : m_coralPitch{coralpitch}, m_elevator{elevator} {


  AddRequirements(m_coralPitch);
}

void GoToL4::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
m_coralPitch->SetCoralIntakeAngle(IntakeConstants::kPitchToL4);
m_elevator ->GoToSetPoint(ElevatorConstants::kElevatorL4Height);
}

bool GoToL4::IsFinished(){
    if (abs(IntakeConstants::kPitchToL4 - m_coralPitch->GetCoralIntakeAngle()) < 0.01 && abs(ElevatorConstants::kElevatorL4Height - m_elevator->GetElevatorPosition()) < 0.01)
        return true;
    else
        return false; 
}

void GoToL4::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}