#include "Constants.h"
#include "commands/autocommands/WristToIntake.h"

SetWristToIntake::SetWristToIntake(CoralPitchSubsystem *coralpitch) : m_coralPitch{coralpitch} {


  AddRequirements(m_coralPitch);
}

void SetWristToIntake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
m_coralPitch->SetCoralIntakeAngle(IntakeConstants::kPitchL1Angle);
}

bool SetWristToIntake::IsFinished(){
    if (abs(IntakeConstants::kPitchL1Angle - m_coralPitch->GetCoralIntakeAngle()) < 0.001)
        return true;
    else
        return false; 
}

void SetWristToIntake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}