#include "Constants.h"
#include "commands/coralintake/GoToIntake.h"

GoToIntake::GoToIntake(CoralPitchSubsystem *coralpitch, ElevatorSubsystem *elevator) : m_coralPitch{coralpitch}, m_elevator{elevator} {


  AddRequirements(m_coralPitch);
}

void GoToIntake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
m_coralPitch->SetCoralIntakeAngle(IntakeConstants::kPitchToIntake);
m_elevator ->GoToSetPoint(ElevatorConstants::kElevatorIntakeHeight);
}

bool GoToIntake::IsFinished(){
    if (abs(IntakeConstants::kPitchL1Angle - m_coralPitch->GetCoralIntakeAngle()) < 0.01 && abs(ElevatorConstants::kElevatorIntakeHeight - m_elevator->GetElevatorPosition()) < 0.01)
        return true;
    else
        return false; 
}

void GoToIntake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}