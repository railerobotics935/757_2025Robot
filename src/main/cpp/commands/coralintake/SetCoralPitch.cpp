
#include "Constants.h"
#include "commands/intake/SetPitch.h"

SetPitch::SetPitch(IntakeSubsystem* intake, frc::XboxController* operatorController) 
: m_intake{intake}, m_operatorController{operatorController} {

  AddRequirements(m_intake);
}

void SetPitch::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SetPitch Initialized\r\n";
#endif

//  m_intake->SetMotorPower(1.0);
}

void SetPitch::Execute() {
  m_intake->SetPitchPosition(units::radian_t(std::numbers::pi / 4));
}

void SetPitch::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SetPitch Ended\r\n";
#endif

  m_intake->SetPitchPower(0.0);
}