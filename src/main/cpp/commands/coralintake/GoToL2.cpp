#include "Constants.h"
#include "commands/coralintake/GoToL2.h"

GoToL2::GoToL2(CoralPitchSubsystem *coralpitch, ElevatorSubsystem *elevator) : m_coralPitch{coralpitch}, m_elevator{elevator} {


  AddRequirements(m_coralPitch);
}

void GoToL2::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
m_coralPitch->SetCoralIntakeAngle(IntakeConstants::kPitchL2Angle);
m_elevator ->GoToSetPoint(ElevatorConstants::kElevatorL2Height);
}

bool GoToL2::IsFinished(){
    if (abs(IntakeConstants::kPitchL2Angle - m_coralPitch->GetCoralIntakeAngle()) < 0.01 && abs(ElevatorConstants::kElevatorL2Height - m_elevator->GetElevatorPosition()) < 0.01)
        return true;
    else
        return false; 
}

void GoToL2::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}