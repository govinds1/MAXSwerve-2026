// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveAutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Helpers;
import frc.robot.controllers.DriverController;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      new Pose2d()
    );

    // PID Controllers
    public PIDController m_xController = new PIDController(DriveAutoConstants.kPXYController, 0, 0);
    public PIDController m_yController = new PIDController(DriveAutoConstants.kPXYController, 0, 0);
    public ProfiledPIDController m_thetaController = new ProfiledPIDController(
      DriveAutoConstants.kPThetaController,
      0, 0,
      DriveAutoConstants.kThetaControllerConstraints
    );
    public HolonomicDriveController m_robotDriveController;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // From PathPlanner Example:
    // Load RobotConfig.
    try{
      //DriveConstants.config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      //e.printStackTrace();
    }

    // Configure AutoBuilder last
    /*AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(DriveAutoConstants.kPXYController, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(DriveAutoConstants.kPThetaController, 0.0, 0.0) // Rotation PID constants
            ),
            DriveConstants.config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );*/

    m_thetaController.enableContinuousInput(0, 2 * Math.PI);

    m_robotDriveController = new HolonomicDriveController(
        m_xController,
        m_yController,
        m_thetaController
    );
    // Set controller tolerance.
    m_robotDriveController.setTolerance(DriveAutoConstants.kRobotControllerTolerance);
  }

  // TODO: Need init? Probably need to set gyro based off alliance start. If red, need to add 180 degrees to gryo at all times.

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(),
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    // Update Pose with Limelight if possible.
    //localizePose();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Localize pose with AprilTags and Limelight's MegaTag2 localizer. Call this function whenver an AprilTag is found.
   *
   * @returns true if we updated pose successfully.
   */
  public boolean localizePose() {
    LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    
    // if our angular velocity is greater than 360 degrees per second, ignore vision updates
    boolean doRejectUpdate = true;
    if(Math.abs(getTurnRate()) > 360)
    {
      doRejectUpdate = true;
    }
    if(mt2.tagCount == 0)
    {
      doRejectUpdate = true;
    }
    if(!doRejectUpdate)
    {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      // TODO: Ignore mt2.pose if it's too far from current pose? 
      m_poseEstimator.addVisionMeasurement(
          mt2.pose,
          mt2.timestampSeconds);
    }

    return !doRejectUpdate;
  }

  /**
   * Method to drive the robot given joystick.
   *
   * @param controller    Controller object to grab teleop driver's desired inputs.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void driveWithJoystick(DriverController controller) {
    driveWithJoystick(controller, new ChassisSpeeds());
  }

  /**
   * Method to drive the robot given joystick.
   *
   * @param controller    Controller object to grab teleop driver's desired inputs.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param offset        ChassisSpeeds offset velocity to apply.
   */
  public void driveWithJoystick(DriverController controller, ChassisSpeeds offset) {
    // Get driver controller inputs.
    double forward = -controller.getLeftY(); // Pushing forward on stick Y axis is negative, so invert to get forward speed.
    double left = -controller.getLeftX();    // Pushing right on stick X axis is positive, so invert to get left speed.
    double rotCCW = -controller.getRightX();  // Pushing right on stick X axis is positive, so invert to get CCW speed.
    // Apply deadbands.
    forward = MathUtil.applyDeadband(forward, 0.05);
    left = MathUtil.applyDeadband(left, 0.05);
    rotCCW = MathUtil.applyDeadband(rotCCW, 0.05);
    // Square inputs.
    forward = Helpers.signedSquare(forward);
    left = Helpers.signedSquare(left);
    rotCCW = Helpers.signedSquare(rotCCW);
    // Drive robot manually.
    drive(forward, left, rotCCW, true, offset);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward +).
   * @param ySpeed        Speed of the robot in the y direction (left +).
   * @param rot           Angular rate of the robot (ccw +).
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    drive(xSpeed, ySpeed, rot, fieldRelative, new ChassisSpeeds());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward +).
   * @param ySpeed        Speed of the robot in the y direction (left +).
   * @param rot           Angular rate of the robot (ccw +).
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param offset        ChassisSpeeds offset velocity to apply.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, ChassisSpeeds offset) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = (xSpeed * DriveConstants.kMaxSpeedMetersPerSecond) + offset.vxMetersPerSecond;
    double ySpeedDelivered = (ySpeed * DriveConstants.kMaxSpeedMetersPerSecond) + offset.vyMetersPerSecond;
    double rotDelivered = (rot * DriveConstants.kMaxAngularSpeedRadiansPerSecond) + offset.omegaRadiansPerSecond;

    // Calculate robot relative ChassisSpeeds
    ChassisSpeeds newSpeeds;
    if (fieldRelative) {
      newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getHeadingRotation());
    } else {
      newSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    }

    // Drive robot to new ChassisSpeeds
    driveRobotRelative(newSpeeds);
  }

  //TODO: 
  public ChassisSpeeds getChassisSpeedsForDriveInput(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeedRadiansPerSecond;

    // Calculate robot relative ChassisSpeeds
    ChassisSpeeds newSpeeds;
    if (fieldRelative) {
      newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getHeadingRotation());
    } else {
      newSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    }
    return newSpeeds;
  }

  /**
   * Drives the robot given robot relative ChassisSpeeds
   * 
   * @param newSpeeds ChassisSpeeds to drive the robot at.
   */
  public void driveRobotRelative(ChassisSpeeds newSpeeds) {
    // Calculate SwerveModuleStates given desired robot relative ChassisSpeeds
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(newSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void stop() {
    drive(0, 0, 0, false);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeadingDegrees() {
    return getHeadingRotation().getDegrees();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading as a Rotation2d
   */
  public Rotation2d getHeadingRotation() {
    // TODO: Do we need to normalize this angle to 0 to 360 or -180 to 180?;
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ));
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the current velocities of the robot.
   * 
   * @return The robot relative velocities, as ChassisSpeeds object
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    );
  }
}