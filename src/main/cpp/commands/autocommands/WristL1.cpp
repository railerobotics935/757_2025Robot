#include "Constants.h"
#include "commands/autocommands/WristL1.h"

SetWristToL1::SetWristToL1(CoralPitchSubsystem *coralpitch) : m_coralPitch{coralpitch} {


  AddRequirements(m_coralPitch);
}

void SetWristToL1::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
m_coralPitch->SetCoralIntakeAngle(IntakeConstants::kPitchToIntake);
}

bool SetWristToL1::IsFinished(){
    if (abs(IntakeConstants::kPitchToIntake - m_coralPitch->GetCoralIntakeAngle()) < 0.001)
        return true;
    else
        return false; 
}

void SetWristToL1::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}