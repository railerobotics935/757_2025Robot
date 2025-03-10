
#include "Constants.h"
#include "commands/coralintake/SimpleCoralIntake.h"

SimpleCoralIntake::SimpleCoralIntake(CoralIntakeSubsystem* intake, frc::XboxController* operatorController) 
: m_coralIntake{intake}, m_operatorController{operatorController} {

  AddRequirements(m_coralIntake);
}

void SimpleCoralIntake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif

//  m_intake->SetMotorPower(1.0);
}

void SimpleCoralIntake::Execute() {

  const auto coralIntakeSpeed = -frc::ApplyDeadband(m_operatorController->GetRawAxis(ControllerConstants::kCoralIntakeButton), 0.05);
  const auto coralOuttakeSpeed = frc::ApplyDeadband(m_operatorController->GetRawAxis(ControllerConstants::kCoralOuttakeButton), 0.05);

  if (coralIntakeSpeed < 0) {
    m_coralIntake->SetCoralIntakeMotorPower(m_coralIntake->SignedSquare(coralIntakeSpeed));
  }
  else if (coralOuttakeSpeed > 0) {
    m_coralIntake->SetCoralIntakeMotorPower(m_coralIntake->SignedSquare(coralOuttakeSpeed));
  }
  else {
    m_coralIntake->SetCoralIntakeMotorPower(0);
  }

}

bool SimpleCoralIntake::IsFinished() {
  if(m_coralIntake->CoralInIntake() && m_coralIntake->GetDirection() < 0) {
    return true;
  }
  else {
    return false;
  }
}

void SimpleCoralIntake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

  m_coralIntake->SetCoralIntakeMotorPower(0.0);
}