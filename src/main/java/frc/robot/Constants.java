// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI; // radians per second

    // TODO: Modify chassis config lengths.
    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5); // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5); // Distance between front and back wheels on robot
    public static final double kBumperWidth = Units.inchesToMeters(3); // Width of a single bumper.
    public static final double kFullWidth = Units.inchesToMeters(27) + (kBumperWidth * 2.0); // Width from bumper to bumper
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // Spark Max CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;

    // RobotConfig for PathPlanner
    public static RobotConfig config;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    // PID Values
    public static final double kDrivingPController = 0.04;
    public static final double kTurningPController = 1;
  }

  public static final class ShooterConstants {
    public static final int kLeftShooterMotorCanId = 21;
    public static final int kRightShooterMotorCanId = 22;
    public static final int kFeederMotorCanId = 23;
    public static final double kShooterPower = 0.55;
    public static final double kFeederPower = 0.6;
    public static final boolean kInvertFollower = true;
    public static final double kShooterSpinUpTime = 0.25;

    // TODO: Modify these buttons if necessary.
    public static final int kRunShooterButton = 1; // A button
    public static final int kStopShooterButton = 4; // Y button
    public static final int kRunFeederButton = 5; // Left bumper?

    public static final double kShooterMaxRPM = 600; // TODO:
    public static final double kPController = 5;
    public static final double kBallRadiusMeters = Units.inchesToMeters(3);
    public static final double kLaunchAngleDegrees = 60; // TODO:
    public static final double kLaunchAngleRadians = Math.toRadians(kLaunchAngleDegrees);
    public static final double kLaunchHeightMeters = Units.inchesToMeters(22) + kBallRadiusMeters;
    public static final double kHubHeightMeters = Units.inchesToMeters(72);
    public static final double kShotVerticalDistance = kHubHeightMeters - kLaunchHeightMeters;
    public static final double kMaxShootTime = 4.0;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kFlyWheelDiameterMeters = 0.0762; // TODO:
    public static final double kFlyWheelCircumferenceMeters = kFlyWheelDiameterMeters * Math.PI;
    // TODO: Figure out shooting motor reduction.
    public static final double kMotorReduction = (45.0 * 22) / (10 * 15);
    public static final double kMotorEncoderToRevolutions = kMotorReduction; // TODO: is this the correct factor to get to Revolutions from encoder rotations?
    public static final double kFlyWheelFreeSpeedRps = (kMotorFreeSpeedRps * kFlyWheelCircumferenceMeters)
        / kMotorReduction;
  }

  public static final class IntakeConstants {
    public static final int kIntakeRollerMotorCanId = 31;
    public static final int kIntakeExtenderMotorCanId = 32;
    public static final double kIntakeRollerSpeed = 0.8;
    public static final double kIntakeExtenderMotorSpeed = 0.1;
    public static final double kIntakeExtendTime = 0.5;

    public static final double kIntakeExtenderForwardLimitEncoderUnits = 1000; // TODO: Tune.

    // TODO: Modify these buttons.
    public static final int kRollerRunButton = 2; // B button
    public static final int kRollerReverseButton = 3; // X button
    public static final int kExtenderOutAxis = 2; // Left trigger // TODO: Should extender control be a toggle?
    public static final int kExtenderInAxis = 3; // Right trigger
  }

  public static final class ClimberConstants {
    public static final int kClimberMotorCanId = 41;
    public static final double kClimbSpeed = 0.3;

    // TODO: Modify these buttons.
    public static final int kClimbUpButton = 5; // Left bumper
    public static final int kClimbDownButton = 6; // Right bumper 
  }

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;

    public static final int kResetGyroButton = 7; // Start button
    public static final int kHalfSpeedAxis = 2; // Left trigger
    public static final int kAimAndDriveButton = 1; // A button
  }

  public static final class DriveAutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

    public static final double kPXYController = 5;
    public static final double kPThetaController = 5;
    public static final Pose2d kRobotControllerTolerance = new Pose2d(0.1, 0.1, new Rotation2d(Units.degreesToRadians(1)));

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
    
    public static final double kFindHubMaxTime = 4.0;
    public static final double kAimAtHubMaxTime = 5.0;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class LimelightConstants {
    // Camera's pose in robot space.
    public static final double kForwardOffsetMeters = 0.5;
    public static final double kSideOffsetMeters = 0.0;
    public static final double kHeightOffsetMeters = 0.5;
    public static final double kRollDegrees = 0.0;
    public static final double kPitchDegrees = 30.0;
    public static final double kYawDegrees = 0.0;
    public static final double kCameraLatencySeconds = 0.08; // Should adjust this empirically.
  }

  public static final class AprilTagConstants {
    public static enum TagLocation {
      kTrenchRightClose, kTrenchRightFar, kTrenchLeftClose, kTrenchLeftFar,
      kOutpost, kTower, kHubRight, kHubFar, kHubLeft, kHubClose
    }
    public static class Tag {
      public TagLocation m_location;
      public Alliance m_alliance;
      public int m_id;
      Tag(TagLocation location, Alliance alliance, int id) {
        m_location = location;
        m_alliance = alliance;
        m_id = id;
      }
    }
    public static final List<Tag> tags = List.of(
      new Tag(TagLocation.kTrenchRightFar, Alliance.Red, 1),
      new Tag(TagLocation.kHubRight, Alliance.Red, 2),
      new Tag(TagLocation.kHubFar, Alliance.Red, 3),
      new Tag(TagLocation.kHubFar, Alliance.Red, 4),
      new Tag(TagLocation.kHubLeft, Alliance.Red, 5),
      new Tag(TagLocation.kTrenchLeftFar, Alliance.Red, 6),
      new Tag(TagLocation.kTrenchLeftClose, Alliance.Red, 7),
      new Tag(TagLocation.kHubLeft, Alliance.Red, 8),
      new Tag(TagLocation.kHubClose, Alliance.Red, 9),
      new Tag(TagLocation.kHubClose, Alliance.Red, 10),
      new Tag(TagLocation.kHubRight, Alliance.Red, 11),
      new Tag(TagLocation.kTrenchRightClose, Alliance.Red, 12),
      new Tag(TagLocation.kOutpost, Alliance.Red, 13),
      new Tag(TagLocation.kOutpost, Alliance.Red, 14),
      new Tag(TagLocation.kTower, Alliance.Red, 15),
      new Tag(TagLocation.kTower, Alliance.Blue, 16),
      new Tag(TagLocation.kTrenchRightFar, Alliance.Blue, 17),
      new Tag(TagLocation.kHubRight, Alliance.Blue, 18),
      new Tag(TagLocation.kHubFar, Alliance.Blue, 19),
      new Tag(TagLocation.kHubFar, Alliance.Blue, 20),
      new Tag(TagLocation.kHubLeft, Alliance.Blue, 21),
      new Tag(TagLocation.kTrenchLeftFar, Alliance.Blue, 22),
      new Tag(TagLocation.kTrenchLeftClose, Alliance.Blue, 23),
      new Tag(TagLocation.kHubLeft, Alliance.Blue, 24),
      new Tag(TagLocation.kHubClose, Alliance.Blue, 25),
      new Tag(TagLocation.kHubClose, Alliance.Blue, 26),
      new Tag(TagLocation.kHubRight, Alliance.Blue, 27),
      new Tag(TagLocation.kTrenchRightClose, Alliance.Blue, 28),
      new Tag(TagLocation.kOutpost, Alliance.Blue, 29),
      new Tag(TagLocation.kOutpost, Alliance.Blue, 30),
      new Tag(TagLocation.kTower, Alliance.Blue, 31),
      new Tag(TagLocation.kTower, Alliance.Blue, 32)
    );
  }

  public static final class FieldConstants {
    // Locations of field landmarks relative to origin (BOTTOM LEFT CORNER OF FIELD)
    // Origin is marked by PathPlanner as the bottom of the blue alliance wall.
    // https://www.chiefdelphi.com/t/pathplanner-2024-beta/442364/370
    // X is forward axis, positive means FURTHER from the BLUE driver station (all landmarks will be positive)
    // Y is left axis, positive means towards the left side of the field when looking from our driver station. (all landmarks will be positive)
    // All units are in meters. Conversion factor from inches to meters is 0.0254.

    // Important Field locations
    private static final Translation2d kCenterOfField = new Translation2d(Units.inchesToMeters(325.06), Units.inchesToMeters(158.32));
    //private static final double kCenterXMeters = kCenterOfField.getX();
    private static final double kCenterYMeters = kCenterOfField.getY();
    public static final Translation2d kBlueHub = new Translation2d(Units.inchesToMeters(181.56), kCenterYMeters);
    public static final Translation2d kBlueDepot = new Translation2d(Units.inchesToMeters(13.50), kCenterYMeters + (Units.inchesToMeters(75.93)));
    public static final Translation2d kBlueOutpost = new Translation2d(0, Units.inchesToMeters(26.22));
    public static final Translation2d kBlueTower = new Translation2d(Units.inchesToMeters(41.56), kCenterYMeters - (Units.inchesToMeters(11.38)));
    public static final Translation2d kBlueLeftTrench = new Translation2d(kBlueHub.getX(), Units.inchesToMeters(316.62 - 25.62));
    public static final Translation2d kBlueRightTrench = new Translation2d(kBlueHub.getX(), Units.inchesToMeters(25.62));
    public static final Translation2d kRedHub = kBlueHub.rotateAround(kCenterOfField, new Rotation2d(Math.PI));
    public static final Translation2d kRedDepot = kBlueDepot.rotateAround(kCenterOfField, new Rotation2d(Math.PI));
    public static final Translation2d kRedOutpost = kBlueOutpost.rotateAround(kCenterOfField, new Rotation2d(Math.PI));
    public static final Translation2d kRedTower = kBlueTower.rotateAround(kCenterOfField, new Rotation2d(Math.PI));
    public static final Translation2d kRedLeftTrench = kBlueLeftTrench.rotateAround(kCenterOfField, new Rotation2d(Math.PI));
    public static final Translation2d kRedRightTrench = kBlueRightTrench.rotateAround(kCenterOfField, new Rotation2d(Math.PI));

    public static final double kHubHeightMeters = 2.0;

    // Robot starting poses, marking the center of the robot.
    private static final double kRobotPoseXAtBlueStartLine = Units.inchesToMeters(156.61 - (DriveConstants.kFullWidth / 2.0));
    private static final Rotation2d kRobotRotAtStartLine = new Rotation2d(Math.PI);
    public static final Pose2d kBlueLeftStart = new Pose2d(kRobotPoseXAtBlueStartLine, kCenterYMeters + Units.inchesToMeters(75.93), kRobotRotAtStartLine);
    public static final Pose2d kBlueMiddleStart = new Pose2d(kRobotPoseXAtBlueStartLine, kCenterYMeters, kRobotRotAtStartLine);
    public static final Pose2d kBlueRightStart = new Pose2d(kRobotPoseXAtBlueStartLine, kCenterYMeters - Units.inchesToMeters(96.5) - (DriveConstants.kFullWidth / 2.0), kRobotRotAtStartLine);
    public static final Pose2d kRedLeftStart = kBlueLeftStart.rotateAround(kCenterOfField, new Rotation2d(Math.PI));
    public static final Pose2d kRedMiddleStart = kBlueMiddleStart.rotateAround(kCenterOfField, new Rotation2d(Math.PI));
    public static final Pose2d kRedRightStart = kBlueRightStart.rotateAround(kCenterOfField, new Rotation2d(Math.PI));
  }
}