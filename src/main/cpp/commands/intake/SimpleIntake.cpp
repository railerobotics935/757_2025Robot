#include "Constants.h"
#include "commands/intake/SimpleIntake.h"

SimpleIntake::SimpleIntake(IntakeSubsystem* intake, frc::XboxController* operatorController) 
: m_intake{intake}, m_operatorController{operatorController} {

  AddRequirements(m_intake);
}

void SimpleIntake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif

//  m_intake->SetMotorPower(1.0);
}

void SimpleIntake::Execute() {

  const auto intakeSpeed = -frc::ApplyDeadband(m_operatorController->GetRawAxis(ControllerConstants::kIntakeTrigger), 0.05);
  const auto outtakeSpeed = frc::ApplyDeadband(m_operatorController->GetRawAxis(ControllerConstants::kOuttakeTrigger), 0.05);

  if (intakeSpeed < 0) {
    m_intake->SetMotorPower(m_intake->SignedSquare(intakeSpeed));
  }
  else if (outtakeSpeed > 0) {
    m_intake->SetMotorPower(m_intake->SignedSquare(outtakeSpeed));
  }
  else {
    m_intake->SetMotorPower(0);
  }

}

bool SimpleIntake::IsFinished() {
  if(m_intake->CoralInIntake()) {
    return true;
  }
  else {
    return false;
  }
}

void SimpleIntake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

  m_intake->SetMotorPower(0.0);
}