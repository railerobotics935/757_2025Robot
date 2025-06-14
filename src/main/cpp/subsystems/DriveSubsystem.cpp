// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>

#include "pathplanner/lib/auto/AutoBuilder.h"
#include "pathplanner/lib/commands/FollowPathCommand.h"
//#include "pathplanner/lib/util/PIDConstants.h"
#include "pathplanner/lib/controllers/PPHolonomicDriveController.h"

#include "Constants.h"
#include "utils/SwerveUtils.h"

using namespace DriveConstants;

using namespace pathplanner;

DriveSubsystem::DriveSubsystem()
  : m_frontLeft{kFrontLeftDriveMotorPort,
                kFrontLeftTurningMotorPort,
                kFrontLeftCANCoderId},

    m_frontRight{
        kFrontRightDriveMotorPort,       
        kFrontRightTurningMotorPort,
        kFrontRightCANCoderId},
    
    m_backLeft{
        kBackLeftDriveMotorPort,       
        kBackLeftTurningMotorPort,
        kBackLeftCANCoderId},

    m_backRight{
        kBackRightDriveMotorPort,       
        kBackRightTurningMotorPort,  
        kBackRightCANCoderId},

    m_odometry{m_driveKinematics,
                m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw),
                {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                m_backLeft.GetPosition(), m_backRight.GetPosition()},
                frc::Pose2d{(units::meter_t)3.0, (units::meter_t)3.0, m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw)}},

    m_poseEstimator{m_driveKinematics,
                m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw),
                {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                m_backLeft.GetPosition(), m_backRight.GetPosition()},
                frc::Pose2d{(units::meter_t)3.0, (units::meter_t)3.0, m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw)},
                {0.05, 0.05, 0.001}, // Standard Deviation of the encoder position value
                {0.2, 0.2, 0.05}} // Standard Deviation of vision pose esitmation

                
{

RobotConfig config = RobotConfig::fromGUISettings();

AutoBuilder::configure(
        [this](){ return GetOdometryPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](auto speeds, auto feedforwards){ DriveWithChassisSpeeds(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            PIDConstants(AutoConstants::kPTanslationController, AutoConstants::kPTanslationController, AutoConstants::kDTanslationController), // Translation PID constants
            PIDConstants(AutoConstants::kPRotationController, AutoConstants::kIRotationController, AutoConstants::kDRotationController) // Rotation PID constants
        ),
        config, // The robot configuration
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            auto alliance = frc::DriverStation::GetAlliance(); 

            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );


  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("datatable");

  nte_fl_set_angle = nt_table->GetEntry("Swerve Drive/Front Left/Set Angle");
  nte_fr_set_angle = nt_table->GetEntry("Swerve Drive/Front Right/Set Angle");
  nte_bl_set_angle = nt_table->GetEntry("Swerve Drive/Back Left/Set Angle");
  nte_br_set_angle = nt_table->GetEntry("Swerve Drive/Back Right/Set Angle");
  nte_fl_set_speed = nt_table->GetEntry("Swerve Drive/Front Left/Set Speed");
  nte_fr_set_speed = nt_table->GetEntry("Swerve Drive/Front Right/Set Speed");
  nte_bl_set_speed = nt_table->GetEntry("Swerve Drive/Back Left/Set Speed");
  nte_br_set_speed = nt_table->GetEntry("Swerve Drive/Back Right/Set Speed");

  nte_fl_real_angle = nt_table->GetEntry("Swerve Drive/Front Left/Real Angle");
  nte_fr_real_angle = nt_table->GetEntry("Swerve Drive/Front Right/Real Angle");
  nte_bl_real_angle = nt_table->GetEntry("Swerve Drive/Back Left/Real Angle");
  nte_br_real_angle = nt_table->GetEntry("Swerve Drive/Back Right/Real Angle");
  nte_fl_real_speed = nt_table->GetEntry("Swerve Drive/Front Left/Real Speed");
  nte_fr_real_speed = nt_table->GetEntry("Swerve Drive/Front Right/Real Speed");
  nte_bl_real_speed = nt_table->GetEntry("Swerve Drive/Back Left/Real Speed");
  nte_br_real_speed = nt_table->GetEntry("Swerve Drive/Back Right/Real Speed");

  nte_fl_encoder_position = nt_table->GetEntry("Swerve Drive/Front Left/Encoder Position");
  nte_fr_encoder_position = nt_table->GetEntry("Swerve Drive/Front Right/Encoder Position");
  nte_bl_encoder_position = nt_table->GetEntry("Swerve Drive/Back Left/Encoder Position");
  nte_br_encoder_position = nt_table->GetEntry("Swerve Drive/Back Right/Encoder Position");

  nte_gyro_angle = nt_table->GetEntry("Swerve Drive/Gyro Angle");
  nte_robot_x = nt_table->GetEntry("Swerve Drive/Robot X");
  nte_robot_y = nt_table->GetEntry("Swerve Drive/Robot Y");

  nte_ktp = nt_table->GetEntry("Swerve Drive/Turning/ktP");
  nte_kti = nt_table->GetEntry("Swerve Drive/Turning/ktI");
  nte_ktd = nt_table->GetEntry("Swerve Drive/Turning/ktD"); 

  nte_ktp.SetDouble(ModuleConstants::kTurningP);
  nte_kti.SetDouble(ModuleConstants::kTurningI);
  nte_ktd.SetDouble(ModuleConstants::kTurningD);

  ktp_sub = nt_table->GetDoubleTopic("Swerve Drive/Turning/ktP").Subscribe(ModuleConstants::kTurningP);
  kti_sub = nt_table->GetDoubleTopic("Swerve Drive/Turning/ktI").Subscribe(ModuleConstants::kTurningI);
  ktd_sub = nt_table->GetDoubleTopic("Swerve Drive/Turning/ktD").Subscribe(ModuleConstants::kTurningD);

  nte_kdp = nt_table->GetEntry("Swerve Drive/Driving/kdP");
  nte_kdi = nt_table->GetEntry("Swerve Drive/Driving/kdI");
  nte_kdd = nt_table->GetEntry("Swerve Drive/Driving/kdD"); 

  nte_kdp.SetDouble(ModuleConstants::kDrivingP);
  nte_kdi.SetDouble(ModuleConstants::kDrivingI);
  nte_kdd.SetDouble(ModuleConstants::kDrivingD);

  kdp_sub = nt_table->GetDoubleTopic("Swerve Drive/Driving/kdP").Subscribe(ModuleConstants::kDrivingP);
  kdi_sub = nt_table->GetDoubleTopic("Swerve Drive/Driving/kdI").Subscribe(ModuleConstants::kDrivingI);
  kdd_sub = nt_table->GetDoubleTopic("Swerve Drive/Driving/kdD").Subscribe(ModuleConstants::kDrivingD);

  nte_debugTimeForPoseEstimation = nt_table->GetEntry("Debug Values/Pose Estimation");
  nte_debugTimeForAddVistionData = nt_table->GetEntry("Debug Values/Add Vision Data");  
  nte_numberOfTagsAdded = nt_table->GetEntry("Debug Values/Number Of Tags Processed");

  //nte_ktp.SetDouble(2.5);
  //nte_kti.SetDouble(0.002);
  //nte_ktd.SetDouble(0.05);

  
  // Send Field to shuffleboard
  frc::Shuffleboard::GetTab("Field").Add(m_field);

  m_robotAngleController.EnableContinuousInput(0, (std::numbers::pi * 2));

  m_timer.Restart();
}

// Returns true is the allience selected is red
bool DriveSubsystem::InRedAlliance() {
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    return false;
  else
    return true;
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), 
                    m_backLeft.GetPosition(), m_backRight.GetPosition()});

  //m_poseEstimator.Update(m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw), 
                      //{m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_backLeft.GetPosition(), m_backRight.GetPosition()});

  // set odometry relative to the apriltag
 // if (GetLinearRobotSpeed() < 1.0 && GetTurnRate() < 20.0)
   // EstimatePoseWithApriltag();
  
  UpdateNTE();
  GetTurningPIDParameters();
  GetDrivingPIDParameters();

  //m_field.SetRobotPose(m_poseEstimator.GetEstimatedPosition());

  //m_robotAngleController.SetP(nte_ktp.GetDouble(4.5));
  //m_robotAngleController.SetI(nte_kti.GetDouble(0.002));
  //m_robotAngleController.SetD(nte_ktd.GetDouble(0.05));
}
// This updates the Network table entries
void DriveSubsystem::UpdateNTE() {

  nte_fl_real_angle.SetDouble((double)m_frontLeft.GetState().angle.Radians());
  nte_fr_real_angle.SetDouble((double)m_frontRight.GetState().angle.Radians());
  nte_bl_real_angle.SetDouble((double)m_backLeft.GetState().angle.Radians());
  nte_br_real_angle.SetDouble((double)m_backRight.GetState().angle.Radians());
  nte_fl_real_speed.SetDouble((double)m_frontLeft.GetState().speed);
  nte_fr_real_speed.SetDouble((double)m_frontRight.GetState().speed);
  nte_bl_real_speed.SetDouble((double)m_backLeft.GetState().speed);
  nte_br_real_speed.SetDouble((double)m_backRight.GetState().speed);

  nte_gyro_angle.SetDouble((double)m_odometry.GetPose().Rotation().Radians());
  nte_robot_x.SetDouble((double)m_odometry.GetPose().X());
  nte_robot_y.SetDouble((double)m_odometry.GetPose().Y());



  nte_fl_encoder_position.SetDouble((double)m_frontLeft.GetPosition().angle.Radians());
  nte_fr_encoder_position.SetDouble((double)m_frontRight.GetPosition().angle.Radians());
  nte_bl_encoder_position.SetDouble((double)m_backLeft.GetPosition().angle.Radians());
  nte_br_encoder_position.SetDouble((double)m_backRight.GetPosition().angle.Radians());

  // Set robot position to shuffleboard field :)
  m_field.SetRobotPose(GetOdometryPose());



}

void DriveSubsystem::GetTurningPIDParameters() {
  double pValue = ktp_sub.Get();
  double iValue = kti_sub.Get();
  double dValue = ktd_sub.Get();

  if ((pValue != m_turning_Kp) ||
      (iValue != m_turning_Ki) ||
      (dValue != m_turning_Kd)) {
    m_turning_Kp = pValue;
    m_turning_Ki = iValue;
    m_turning_Kd = dValue;
    m_frontLeft.SetTurningPID(m_turning_Kp, m_turning_Ki, m_turning_Kd);
    m_frontRight.SetTurningPID(m_turning_Kp, m_turning_Ki, m_turning_Kd);
    m_backLeft.SetTurningPID(m_turning_Kp, m_turning_Ki, m_turning_Kd);
    m_backRight.SetTurningPID(m_turning_Kp, m_turning_Ki, m_turning_Kd);

  }
}

void DriveSubsystem::GetDrivingPIDParameters() {
  double pValue = kdp_sub.Get();
  double iValue = kdi_sub.Get();
  double dValue = kdd_sub.Get();

  if ((pValue != m_driving_Kp) ||
      (iValue != m_driving_Ki) ||
      (dValue != m_driving_Kd)) {
    m_driving_Kp = pValue;
    m_driving_Ki = iValue;
    m_driving_Kd = dValue;

    m_frontLeft.SetDrivingPID(m_driving_Kp, m_driving_Ki, m_driving_Kd);
    m_frontRight.SetDrivingPID(m_driving_Kp, m_driving_Ki, m_driving_Kd);
    m_backLeft.SetDrivingPID(m_driving_Kp, m_driving_Ki, m_driving_Kd);
    m_backRight.SetDrivingPID(m_driving_Kp, m_driving_Ki, m_driving_Kd);

  }
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool rateLimit) {
                             
  double xSpeedCommanded;
  double ySpeedCommanded;

  if (rateLimit) {
    // Convert XY to polar for rate limitingd
    double inputTranslationDir = atan2(ySpeed.value(), xSpeed.value());
    double inputTranslationMag =
        sqrt(pow(xSpeed.value(), 2) + pow(ySpeed.value(), 2));

    // Calculate the direction slew rate based on an estimate of the lateral
    // acceleration
    double directionSlewRate;
    if (m_currentTranslationMag != 0.0) {
      directionSlewRate =
          abs(DriveConstants::kDirectionSlewRate / m_currentTranslationMag);
    } else {
      directionSlewRate = 500.0;  // some high number that means the slew rate
                                  // is effectively instantaneous
    }

    double currentTime = wpi::Now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    double angleDif = SwerveUtils::AngleDifference(inputTranslationDir,
                                                   m_currentTranslationDir);
    //Steps an angle toward a disired target angle.
    if (angleDif < 0.45 * std::numbers::pi) {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
          m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
    } 
    else if (angleDif > 0.85 * std::numbers::pi) {
      if (m_currentTranslationMag > 1e-4) {  // some small number to avoid floating-point errors with equality checking
        // keep currentTranslationDir unchanged
        m_currentTranslationMag = m_magLimiter.Calculate(0.0);
      }
      // Makes the angle into an angle between 0 and 2*pi.
      else {
        m_currentTranslationDir =
            SwerveUtils::WrapAngle(m_currentTranslationDir + std::numbers::pi);
        m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
      }
    }
    // steps an angle towards a disired target angle.
    else {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
          m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.Calculate(0.0);
    }
    m_prevTime = currentTime;
    
    xSpeedCommanded = m_currentTranslationMag * cos(m_currentTranslationDir);
    ySpeedCommanded = m_currentTranslationMag * sin(m_currentTranslationDir);
    m_currentRotation = m_rotLimiter.Calculate(rot.value());
  } 
  else {
    xSpeedCommanded = xSpeed.value();
    ySpeedCommanded = ySpeed.value();
    m_currentRotation = rot.value();
  }
  if (!m_fieldRelative) {
    xSpeedCommanded = xSpeedCommanded; 
    ySpeedCommanded = ySpeedCommanded; 
  }

  // Convert the commanded speeds into the correct units for the drivetrain
  units::meters_per_second_t xSpeedDelivered =
      xSpeedCommanded * DriveConstants::kMaxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeedCommanded * DriveConstants::kMaxSpeed;
  units::radians_per_second_t rotDelivered =
      m_currentRotation * DriveConstants::kMaxAngularSpeed;

  auto states = m_driveKinematics.ToSwerveModuleStates(
      m_fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(units::radian_t{
                    m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}))
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

  m_driveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);

  // Network table entries
  nte_fl_set_angle.SetDouble((double)fl.angle.Radians());
  nte_fr_set_angle.SetDouble((double)fr.angle.Radians());
  nte_bl_set_angle.SetDouble((double)bl.angle.Radians());
  nte_br_set_angle.SetDouble((double)br.angle.Radians());
  nte_fl_set_speed.SetDouble((double)fl.speed);
  nte_fr_set_speed.SetDouble((double)fr.speed);
  nte_bl_set_speed.SetDouble((double)bl.speed);
  nte_br_set_speed.SetDouble((double)br.speed);
  
}
//Drives the robot at given x and y, and it faces the angle given.
void DriveSubsystem::DriveFacingGoal(units::meters_per_second_t xSpeed,
                                      units::meters_per_second_t ySpeed, 
                                      frc::Rotation2d rotation, 
                                      bool rateLimit) {
  double xSpeedCommanded;
  double ySpeedCommanded;

  if (rateLimit) {
    // Convert XY to polar for rate limiting
    double inputTranslationDir = atan2(ySpeed.value(), xSpeed.value());
    double inputTranslationMag =
        sqrt(pow(xSpeed.value(), 2) + pow(ySpeed.value(), 2));

    // Calculate the direction slew rate based on an estimate of the lateral
    // acceleration
    double directionSlewRate;
    if (m_currentTranslationMag != 0.0) {
      directionSlewRate =
          abs(DriveConstants::kDirectionSlewRate / m_currentTranslationMag);
    } else {
      directionSlewRate = 500.0;  // some high number that means the slew rate
                                  // is effectively instantaneous
    }

    double currentTime = wpi::Now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    double angleDif = SwerveUtils::AngleDifference(inputTranslationDir,
                                                   m_currentTranslationDir);
    if (angleDif < 0.45 * std::numbers::pi) {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
          m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
    } 
    else if (angleDif > 0.85 * std::numbers::pi) {
      if (m_currentTranslationMag > 1e-4) {  // some small number to avoid floating-point errors with equality checking
        // keep currentTranslationDir unchanged
        m_currentTranslationMag = m_magLimiter.Calculate(0.0);
      } 
      else {
        m_currentTranslationDir =
            SwerveUtils::WrapAngle(m_currentTranslationDir + std::numbers::pi);
        m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
      }
    } 
    else {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
          m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.Calculate(0.0);
    }
    m_prevTime = currentTime;

    xSpeedCommanded = m_currentTranslationMag * cos(m_currentTranslationDir);
    ySpeedCommanded = m_currentTranslationMag * sin(m_currentTranslationDir);
    m_currentRotation = m_rotLimiter.Calculate(m_robotAngleController.Calculate(
      (double)GetPose().Rotation().Radians(), (double)rotation.Radians()));
      //(double)(GetHeading() * std::numbers::pi / 180.0), (double)rotation.Radians()));

  } 
  else {
    xSpeedCommanded = xSpeed.value();
    ySpeedCommanded = ySpeed.value();
    m_currentRotation = m_robotAngleController.Calculate((double)GetPose().Rotation().Radians(), (double)rotation.Radians());
  }
  if (!m_fieldRelative) {
    xSpeedCommanded = -xSpeedCommanded; 
    ySpeedCommanded = -ySpeedCommanded; 
  }

  // Put limits so we don't go over the rotation speed limit
  if (m_currentRotation > 1.0)
    m_currentRotation = 1.0;
  if (m_currentRotation < -1.0)
    m_currentRotation = -1.0;

  // Convert the commanded speeds into the correct units for the drivetrain
  units::meters_per_second_t xSpeedDelivered =
      xSpeedCommanded * DriveConstants::kMaxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeedCommanded * DriveConstants::kMaxSpeed;
  units::radians_per_second_t rotDelivered =
      m_currentRotation * (units::radians_per_second_t)(30.0 / (2 * std::numbers::pi)); // could limit this to go a slower speed

  auto states = m_driveKinematics.ToSwerveModuleStates(
      m_fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(units::radian_t{
                    m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}))
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

  m_driveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);

  // Network table entries
  //nte_fl_set_angle.SetDouble((double)fl.angle.Radians());
  //nte_fr_set_angle.SetDouble((double)fr.angle.Radians());
  //nte_bl_set_angle.SetDouble((double)bl.angle.Radians());
  //nte_br_set_angle.SetDouble((double)br.angle.Radians());
  //nte_fl_set_speed.SetDouble((double)fl.speed);
  //nte_fr_set_speed.SetDouble((double)fr.speed);
  //nte_bl_set_speed.SetDouble((double)bl.speed);
  //nte_br_set_speed.SetDouble((double)br.speed);
 }

bool DriveSubsystem::AtAngleSetpoint() {
  return m_robotAngleController.AtSetpoint(); 
}

void DriveSubsystem::DriveWithChassisSpeeds(frc::ChassisSpeeds speeds) {
  auto states = m_driveKinematics.ToSwerveModuleStates(speeds);

  m_driveKinematics.DesaturateWheelSpeeds(&states, ModuleConstants::kModuleMaxLinearVelocity);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  m_driveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         ModuleConstants::kModuleMaxLinearVelocity);
  m_frontLeft.SetDesiredState(desiredStates[4]);
  m_frontRight.SetDesiredState(desiredStates[3]);
  m_backLeft.SetDesiredState(desiredStates[1]);
  m_backRight.SetDesiredState(desiredStates[2]);
}

units::degree_t DriveSubsystem::GetHeading() const {
  return m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw);
}

double DriveSubsystem::GetLinearRobotSpeed() {
  // get magnitude of robot speed vector
  return sqrt(pow((double)GetRobotRelativeSpeeds().vx, 2) + pow((double)GetRobotRelativeSpeeds().vy, 2));
}

void DriveSubsystem::ZeroHeading() {
  m_gyro.Reset();
}

void DriveSubsystem::SetRobotRelative() {
  m_fieldRelative = false;
}

void DriveSubsystem::SetFieldRelative() {
  m_fieldRelative = true;
}

double DriveSubsystem::GetTurnRate() {
  return (double)m_gyro.GetRate(frc::ADIS16470_IMU::kYaw);
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_poseEstimator.GetEstimatedPosition();
}

frc::Pose2d DriveSubsystem::GetOdometryPose() {
  return m_odometry.GetPose();
}

frc::ChassisSpeeds DriveSubsystem::GetRobotRelativeSpeeds()
{
  return m_driveKinematics.ToChassisSpeeds(m_frontLeft.GetState(), 
                                           m_frontRight.GetState(),
                                           m_backLeft.GetState(),
                                           m_backRight.GetState());
}

frc::ChassisSpeeds DriveSubsystem::GetFieldRelativeSpeeds()
{
  return frc::ChassisSpeeds{(units::meters_per_second_t)((GetRobotRelativeSpeeds().vx() * std::cos((double)GetPose().Rotation().Radians())) + // Vx componet from the robot X velocity
                                                         (GetRobotRelativeSpeeds().vy() * std::cos((double)GetPose().Rotation().Radians() + (std::numbers::pi/2)))), // Vx componet from the robot Y velocity

                            (units::meters_per_second_t)((GetRobotRelativeSpeeds().vy() * std::sin((double)GetPose().Rotation().Radians() + (std::numbers::pi/2))) + // Vx componet from the robot X velocity
                                                         (GetRobotRelativeSpeeds().vx() * std::sin((double)GetPose().Rotation().Radians()))), // Vx componet from the robot Y velocity

                            (units::radians_per_second_t)GetRobotRelativeSpeeds().omega()}; // Rotational Velocity stays the same
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()},
      pose);
  m_poseEstimator.ResetPosition(      
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()},
      pose);
}

void DriveSubsystem::EstimatePoseWithApriltag() {
#ifdef DEBUGPOSEESTIMATION
  double startEstiamtionTime = (double)m_timer.GetFPGATimestamp();
  int numberOfValidTags = 0;
#endif
  // Iterate through each tag, adding it to the pose estimator if it is tracked
  for (int tag = 1; tag <= 16; tag++ ) { // Check each tag for each camera
  //int tag = 7;
    // Front Camera
    if (m_frontCameraSensor.TagIsTracked(tag) && m_frontCameraSensor.GetTimestamp(tag) > (units::second_t)0.0){
      #ifdef DEBUGPOSEESTIMATION
      double startEstiamtionTime2 = (double)m_timer.GetFPGATimestamp();
      #endif

      m_poseEstimator.AddVisionMeasurement(m_frontCameraSensor.GetFieldRelativePose(tag).ToPose2d(), m_frontCameraSensor.GetTimestamp(tag), m_frontCameraSensor.GetStandardDeviations(tag));
      
      #ifdef DEBUGPOSEESTIMATION
      nte_debugTimeForAddVistionData.SetDouble((double)m_timer.GetFPGATimestamp() - startEstiamtionTime2);
      numberOfValidTags++;
      #endif
    }

    // Back Left Camera
    if (m_backLeftCameraSensor.TagIsTracked(tag) && m_backLeftCameraSensor.GetTimestamp(tag) > (units::second_t)0.0) {
      #ifdef DEBUGPOSEESTIMATION
      double startEstiamtionTime2 = (double)m_timer.GetFPGATimestamp();
      #endif
      
      m_poseEstimator.AddVisionMeasurement(m_backLeftCameraSensor.GetFieldRelativePose(tag).ToPose2d(), m_backLeftCameraSensor.GetTimestamp(tag), m_backLeftCameraSensor.GetStandardDeviations(tag));
      
      #ifdef DEBUGPOSEESTIMATION
      nte_debugTimeForAddVistionData.SetDouble((double)m_timer.GetFPGATimestamp() - startEstiamtionTime2);
      numberOfValidTags++;
      #endif
    }

    // Back Right Camera
    if (m_backRightCameraSensor.TagIsTracked(tag) && m_backRightCameraSensor.GetTimestamp(tag) > (units::second_t)0.0) {
      #ifdef DEBUGPOSEESTIMATION
      double startEstiamtionTime2 = (double)m_timer.GetFPGATimestamp();
      #endif
      
      m_poseEstimator.AddVisionMeasurement(m_backRightCameraSensor.GetFieldRelativePose(tag).ToPose2d(), m_backRightCameraSensor.GetTimestamp(tag), m_backRightCameraSensor.GetStandardDeviations(tag));
      
      #ifdef DEBUGPOSEESTIMATION
      nte_debugTimeForAddVistionData.SetDouble((double)m_timer.GetFPGATimestamp() - startEstiamtionTime2);
      numberOfValidTags++;
      #endif
    }
  }
#ifdef DEBUGPOSEESTIMATION
  nte_numberOfTagsAdded.SetInteger(numberOfValidTags);
  nte_debugTimeForPoseEstimation.SetDouble((double)m_timer.GetFPGATimestamp() - startEstiamtionTime);
#endif
}       
  

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Utility math functions
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
double DriveSubsystem::SignedSquare(double input) {
  if (input < 0.0)
    return -std::pow(input, 2);
  else
    return std::pow(input, 2);
}
