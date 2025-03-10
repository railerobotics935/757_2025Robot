// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/current.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFields.h>
#include <rev/SparkMax.h>
#include <iostream>
#include <rev/config/SparkMaxConfig.h>



// Turn this off when there is no new constants need to be burned onto motorcontrollers
#define BURNMODULESPARKMAX
#define USEXBOXCONTROLLER
//#define PRINTDEBUG
//#define DEBUGPOSEESTIMATION
#define BURNELEVATORSPARKMAX
#define BURNINTAKESPARKMAX

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */


namespace RobotConstants {

constexpr double kVoltageCompentationValue = 11.0;

const units::meter_t kWheelBase =
    0.6795_m;  // Distance between centers of front and back wheels on robot
const units::meter_t kWheelWidth =
    0.51_m; // Distance between centers of left and right wheels on robot

}

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
constexpr units::meters_per_second_t kMaxSpeed = 4.0_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed{2.0 * std::numbers::pi};
constexpr double kDirectionSlewRate = 6.0;   // radians per second
constexpr double kMagnitudeSlewRate = 7.0;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 8.0;  // percent per second (1 = 100%)

// CAN Sparkmax id numbers
constexpr int kFrontLeftTurningMotorPort = 29;
constexpr int kFrontRightTurningMotorPort = 21;
constexpr int kBackLeftTurningMotorPort = 11;
constexpr int kBackRightTurningMotorPort = 19;

constexpr int kFrontLeftDriveMotorPort = 28;
constexpr int kFrontRightDriveMotorPort = 20;
constexpr int kBackLeftDriveMotorPort = 10;
constexpr int kBackRightDriveMotorPort = 18;


//CANCoder id numbers
constexpr int kFrontLeftCANCoderId = 4;
constexpr int kFrontRightCANCoderId = 3;
constexpr int kBackLeftCANCoderId = 1;
constexpr int kBackRightCANCoderId = 2;

// PID Controller for the auto rotation of the robot
constexpr double kRotationP = 2.5;
constexpr double kRotationI = 0.002;
constexpr double kRotationD = 0.2;


// Offsets in radians for the encoders. the first number to to make zero forward, after that we
// subtract an additional pi to make the full range -pi to pi instead of 0 to 2pi

constexpr auto kDriveBaseRadius = 0.4248_m;

}  // namespace DriveConstants

namespace ModuleConstants {
// Through-hole Encoder on Spark MAX frequency-pwm input
// Invert the turning encoder, since the output shaft rotates in the opposite
// direction of the steering motor in the MAXSwerve Module.
constexpr bool kTurningEncoderInverted = true;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps =
    5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter = 0.0953_m;
constexpr units::meter_t kWheelCircumference =
    kWheelDiameter * std::numbers::pi;

// 6.75:1 Gear Ratio for Driving Motors
constexpr double kDrivingMotorReduction = 6.75;
constexpr double kDriveWheelFreeSpeedRps =
    (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
    kDrivingMotorReduction;

constexpr double kDrivingEncoderPositionFactor =
    (kWheelDiameter.value() * std::numbers::pi) /
    kDrivingMotorReduction;  // meters
constexpr double kDrivingEncoderVelocityFactor =
    ((kWheelDiameter.value() * std::numbers::pi) / kDrivingMotorReduction) /
    60.0;  // meters per second

constexpr double kTurningEncoderPositionFactor =
    (2 * std::numbers::pi);  // radians
constexpr double kTurningEncoderVelocityFactor =
    (2 * std::numbers::pi) / 60.0;  // radians per second

constexpr units::radian_t kTurningEncoderPositionPIDMinInput = 0_rad;
constexpr units::radian_t kTurningEncoderPositionPIDMaxInput =
    units::radian_t{kTurningEncoderPositionFactor};

constexpr double kDrivingP = 0.1;
constexpr double kDrivingI = 0.0;
constexpr double kDrivingD = 0.0;
constexpr double kDrivingFF = (1 / kDriveWheelFreeSpeedRps);
constexpr double kDrivingMinOutput = -1;
constexpr double kDrivingMaxOutput = 1;

constexpr double kTurningP = 2.0;
constexpr double kTurningI = 0.01;
constexpr double kTurningD = 0.004; //was originally 0.15
constexpr double kTurningFF = 0;
constexpr double kTurningMinOutput = -1;
constexpr double kTurningMaxOutput = 1;

constexpr rev::spark::SparkMaxConfig::IdleMode kDrivingMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;
constexpr rev::spark::SparkMaxConfig::IdleMode kTurningMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;

constexpr units::ampere_t kDrivingMotorCurrentLimit = 40_A;
constexpr units::ampere_t kTurningMotorCurrentLimit = 20_A;

constexpr auto kModuleMaxAngularVelocity =  std::numbers::pi * 9_rad_per_s;  // radians per second ?????????
constexpr auto kModuleMaxAngularAcceleration = std::numbers::pi * 20_rad_per_s / 1_s;  // radians per second^2
constexpr auto kModuleMaxLinearVelocity = 4.65_mps;
}  // namespace ModuleConstants


namespace AutoConstants {

// Only Constants here are the PID constants. Look in path planner for max veleocty/acceleration constants
// PID Constants for the tranlation (X and Y movement) of the robot during auto
constexpr double kPTanslationController = 0.0;//4.0; // 6.0
constexpr double kITanslationController = 0.0; // 1.7
constexpr double kDTanslationController = 0.0; // 0.0

// PID Constants for the rotation, or Yaw of the robot
constexpr double kPRotationController = 0.0; // 5.0
constexpr double kIRotationController = 0.0; // 0.0
constexpr double kDRotationController = 0.0; // 0.0

}  // namespace AutoConstants

namespace ControllerConstants {

// Controller Constants for X Box Controllers
/**
 * BUTTONS
 * A button - 1
 * B button - 2
 * X button - 3
 * Y button - 4
 * Left Bumper - 5
 * Right Bumper - 6
 * Center Left Button - 7
 * Center Right Button - 8
 * Left Joystick Button - 9
 * Right Joystick Button - 10
 * 
 * AXES
 * Left x-axis - 0, input right creates a positive output
 * Left y-axis - 1, input down creates a positive output
 * Left Trigger - 2, input in creates a positive output
 * Right Trigger - 3, input in creates a positive output
 * Right x-axis - 4, input right creates a positive output
 * Right y-axis - 5, input down creates a positive output
*/

// Axis indexes
constexpr int kDriveLeftYIndex = 1; // An input UP creates a NEGATIVE output
constexpr int kDriveLeftXIndex = 0; // An input RIGHT creates a NEGATIVE output
constexpr int kDriveRightYIndex = 5; // An input UP creates a NEGATIVE output
constexpr int kDriveRightXIndex = 4; // An input RIGHT creates a NEGATIVE output

#ifdef USEXBOXCONTROLLER
constexpr int kOperatorLeftYIndex = 1; // An input UP creates a NEGATIVE output
constexpr int kOperatorRightYIndex = 5; // An input UP creates a NEGATIVE output
constexpr int kIntakeTrigger = 3;
constexpr int kOuttakeTrigger = 2;



#else
constexpr int kOperatorLeftYIndex = 1; // An input UP creates a NEGATIVE output
constexpr int kOperatorRightYIndex = 3; // An input UP creates a NEGATIVE output

#endif
// Drive Controller
constexpr int kFieldRelativeButtonIndex = 7; // CL
constexpr int kRobotRelativeButtonIndex = 8; // CR
constexpr int kResetGyroButtonIndex = 2; // B
constexpr int kRaiseClimberButtonIndex = 6; // RB
constexpr int kLowerClimberButtonIndex = 5; // LB
constexpr int kLockLatchButtonIndex = 3; // X
constexpr int kUnlockLatchButtonIndex = 4; // Y

// Operator Controller
constexpr int kExtendElevatorTrigger = 6; // Pressing it creates a POSITVE output
constexpr int kRetractElevatorTrigger = 5; // Pressing it creates a POSITVE output
constexpr int kElevatorSetPointButton = 3; // X

} // namespace ControllerConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kOperatorControllerPort = 1;
}  // namespace OIConstants

namespace CameraConstants {

// Min and Max standard deviations for the apriltag detetion 
constexpr double kMinStandardDeviation = 0.2;
constexpr double kMaxStandardDeviation = 3.0;

// Max speed allowed for adding vidion measurments to the robot pose esitmator
constexpr double kMaxEstimationSpeed = 0.25; // mps

/**
 * @param distance The raw distance from the apriltag
 * 
 * @return The standard deviation value for the distance
*/
double GetStandardDeviationFromDistance(double distance);

// Pose3d/transformation2d of the camera relative to the robot
// X if forward, Y is Left, Z is up 
namespace FrontCamera {
    const frc::Translation3d kTranlation3d{(units::meter_t)0.250, (units::meter_t)-0.185, (units::meter_t)0.2286};
    const frc::Rotation3d kRotation3d{(units::radian_t)0.0, (units::radian_t)std::numbers::pi / 12, (units::radian_t)0.0};
    const frc::Pose3d kPose3d{kTranlation3d, kRotation3d};
} // namespace FrontCamera

namespace BackLeftCamera {
    const frc::Translation3d kTranlation3d{(units::meter_t)-0.250, (units::meter_t)0.4125, (units::meter_t)0.2286};
    const frc::Rotation3d kRotation3d{(units::radian_t)std::numbers::pi * -0.1116883853, (units::radian_t)std::numbers::pi * 0.1116883853, (units::radian_t)std::numbers::pi * 1.25};
    const frc::Pose3d kPose3d{kTranlation3d, kRotation3d};
} // namespace BackLeftCamera

namespace BackRightCamera {
    const frc::Translation3d kTranlation3d{(units::meter_t)-0.250, (units::meter_t)-0.4125, (units::meter_t)0.2286};
    const frc::Rotation3d kRotation3d{(units::radian_t)std::numbers::pi * 0.1116883853, (units::radian_t)std::numbers::pi * 0.1116883853, (units::radian_t)std::numbers::pi * 0.75};
    const frc::Pose3d kPose3d{kTranlation3d, kRotation3d};
} // namespace BackRightCamera

} // namespace CameraConstants

namespace ElevatorConstants {

constexpr int kID = 22;
constexpr int kLimitSwitchPort = 2;
constexpr int kElevatorSensA = 0;
constexpr int kElevatorSensB = 1;

//constexpr double kElevatorGearRatio = 36.0 * (60.0 / 37.0) * 3.0;
//constexpr double kElevatorPositionFactor = 4.0 * std::numbers::pi / kElevatorGearRatio; // in meters
constexpr double kElevatorPositionFactor = 4.0 * std::numbers::pi; // in meters
constexpr double kElevatorVelocityFactor = kElevatorPositionFactor;

constexpr int kElevatorP = 0;
constexpr int kElevatorI = 0;
constexpr int kElevatorD = 0;
constexpr int kElevatorFF = 0;

constexpr int kElevatorMinOutput = -1;
constexpr int kElevatorMaxOutput = 1;

constexpr double kMaximumHeight = 1.0; // Should be in meters
constexpr double kMinimumHeight = 0.0;

constexpr units::ampere_t kElevatorMotorCurrentLimit = 40_A;
constexpr rev::spark::SparkLowLevel::MotorType kMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;

constexpr rev::spark::SparkMaxConfig::IdleMode kElevatorMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;


} // namespace ElevatorConstants

namespace IntakeConstants {
// Intake motor 
constexpr int kIntakeMotorID = 26;
constexpr int kPitchMotorID = 27;
constexpr int kLightSensorID = 3;

//PID Values
constexpr int kPitchP = 0;
constexpr int kPitchI = 0;
constexpr int kPitchD = 0;
constexpr int kPitchFF = 0;

constexpr int kPitchMinOutput = -1;
constexpr int kPitchMaxOutput = 1;

constexpr units::ampere_t kIntakeMotorCurrentLimit = 40_A;
constexpr rev::spark::SparkLowLevel::MotorType kMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
constexpr rev::spark::SparkMaxConfig::IdleMode kIntakeMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;

// Pitch limits
constexpr double kMinimumAngle = 0;
constexpr double kMaximumAngle = (std::numbers::pi / 4);
} // namespace IntakeConstant

namespace ClimberConstants {
    constexpr int kMotorID = 0;

    constexpr int kServoPort = 0;
    constexpr int kLimitSwitchPort = 5;

    constexpr units::ampere_t kClimberMotorCurrentLimit = 40_A;
    constexpr double kClimberPositionFactor = 4.0 * std::numbers::pi; // in meters

    constexpr rev::spark::SparkLowLevel::MotorType kMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
    constexpr rev::spark::SparkMaxConfig::IdleMode kClimberMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;

    constexpr double kServoUnlockedPosition = 1.0;
    constexpr double kServoLockedPosition = 0.0; 
} // namespace ClimberConstants