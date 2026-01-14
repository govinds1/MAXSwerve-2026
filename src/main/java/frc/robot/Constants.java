// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.config.RobotConfig;

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
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
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

    // Spark Flex CAN IDs
    // TODO: Set IDs.
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
  }

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class DriveAutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

    public static final double kPXYController = 5;
    public static final double kPThetaController = 5;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
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
}