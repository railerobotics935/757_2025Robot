#include "Constants.h"
#include "commands/coralintake/GoToL3.h"

GoToL3::GoToL3(CoralPitchSubsystem *coralpitch, ElevatorSubsystem *elevator) : m_coralPitch{coralpitch}, m_elevator{elevator} {


  AddRequirements(m_coralPitch);
}

void GoToL3::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
m_coralPitch->SetCoralIntakeAngle(IntakeConstants::kPitchL3Angle);
m_elevator ->GoToSetPoint(ElevatorConstants::kElevatorL3Height);
}

bool GoToL3::IsFinished(){
    if (abs(IntakeConstants::kPitchL3Angle - m_coralPitch->GetCoralIntakeAngle()) < 0.01 && abs(ElevatorConstants::kElevatorL3Height - m_elevator->GetElevatorPosition()) < 0.01)
        return true;
    else
        return false; 
}

void GoToL3::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}