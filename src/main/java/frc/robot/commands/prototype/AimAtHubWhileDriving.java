// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.prototype;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Helpers;
import frc.robot.controllers.DriverController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionTargeting;
import frc.robot.subsystems.VisionTargeting.ShootingInfo;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtHubWhileDriving extends Command {
  DriveSubsystem m_drive;
  VisionTargeting m_vision;
  DriverController m_controller;

  ChassisSpeeds m_aimControlOffset = new ChassisSpeeds();

  /** Creates a new AimAndStrafeControl. */
  // This command is designed to last as long as necessary, until interrupted by the driver.
  // It is for teleop FIELD RELATIVE control of the swerve chassis while aiming at the Hub.
  public AimAtHubWhileDriving(DriveSubsystem drive, VisionTargeting vision, DriverController controller) {
    m_drive = drive;
    m_vision = vision;
    m_controller = controller;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.isAutonomous()) {
      // Do not run this command in autonomous!
      this.cancel();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Take driver controller input (for dx and dy only) and use simple PID to turn to face the Hub.

    ShootingInfo shot = m_vision.getHubAimInfo(m_drive.getPose(), m_drive.getRobotRelativeSpeeds());

    // Drive with offset.
    m_drive.driveWithJoystick(m_controller, shot.pose.getRotation());
  }

  public boolean isAimed() {
    return Helpers.chassisSpeedsMagnitude(m_aimControlOffset) < 0.1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
