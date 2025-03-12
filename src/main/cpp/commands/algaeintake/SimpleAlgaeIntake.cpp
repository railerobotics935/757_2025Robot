
#include "Constants.h"
#include "commands/algaeintake/SimpleAlgaeIntake.h"

SimpleAlgaeIntake::SimpleAlgaeIntake(AlgaeIntakeSubsystem* algaeintake, frc::XboxController* operatorController) 
:m_algaeIntake{algaeintake}, m_operatorController{operatorController} {

  AddRequirements(m_algaeIntake);
}

void SimpleAlgaeIntake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif

//  m_intake->SetMotorPower(1.0);
}

void SimpleAlgaeIntake::Execute() {

  const auto algaeIntakeSpeed = -frc::ApplyDeadband(m_operatorController->GetRawAxis(ControllerConstants::kAlgaeIntakeButton), 0.05);
  const auto algaeOuttakeSpeed = frc::ApplyDeadband(m_operatorController->GetRawAxis(ControllerConstants::kAlgaeOuttakeButton), 0.05);

  if (algaeIntakeSpeed < 0) {
    m_algaeIntake->SetAlgaeIntakeMotorPower(m_algaeIntake->SignedSquare(algaeIntakeSpeed));
  }
  else if (algaeOuttakeSpeed > 0) {
    m_algaeIntake->SetAlgaeIntakeMotorPower(m_algaeIntake->SignedSquare(algaeOuttakeSpeed));
  }
  else {
    m_algaeIntake->SetAlgaeIntakeMotorPower(0);
  }

}

bool SimpleAlgaeIntake::IsFinished() {
 /* if(m_algaeIntake->CoralInIntake() && m_intake->GetDirection() < 0) {
    return true;
  }
  else {
    return false;
  }
  */
 return true; //quick fix for a linker error, fix when possible
}

void SimpleAlgaeIntake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

  m_algaeIntake->SetAlgaeIntakeMotorPower(0.0);
}