#include "Constants.h"
#include "commands/autocommands/WristL4.h"

SetWristToL4::SetWristToL4(CoralPitchSubsystem *coralpitch) : m_coralPitch{coralpitch} {


  AddRequirements(m_coralPitch);
}

void SetWristToL4::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
m_coralPitch->SetCoralIntakeAngle(IntakeConstants::kPitchL1Angle);
}

bool SetWristToL4::IsFinished(){
    if (abs(IntakeConstants::kPitchL1Angle - m_coralPitch->GetCoralIntakeAngle()) < 0.001)
        return true;
    else
        return false; 
}

void SetWristToL4::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}