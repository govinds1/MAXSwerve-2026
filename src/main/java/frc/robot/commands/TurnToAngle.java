// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Helpers;
import frc.robot.Constants.DriveAutoConstants;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToAngle extends Command {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;

  private Rotation2d m_targetAngle;

  /** Creates a new TurnToAngle. */
  public TurnToAngle(DriveSubsystem drive, Rotation2d targetAngle, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    m_drive = drive;
    m_targetAngle = targetAngle;
    m_translationXSupplier = xSupplier;
    m_translationYSupplier = ySupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Subsystems/Drive/Auto/TurnToAngle/TargetAngle", Helpers.modDegrees(m_targetAngle.getDegrees()));
    SmartDashboard.putNumber("Subsystems/Drive/Auto/TurnToAngle/CurrentAngle", Helpers.modDegrees(m_drive.getHeadingDegrees()));
    // Get current rotation speed to apply.
    double rotSpeed = TurnToAngle.getRotSpeed(m_drive.getHeadingRotation().getRadians(), m_targetAngle.getRadians(), m_drive.m_thetaController);
    // Apply rotational output to drive, along with strafe inputs from supplier.
    m_drive.drive(m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble(), rotSpeed, true);
  }

  // Make this public static so other commands can use the same control math for turning.
  public static double getRotSpeed(double currentHeadingRadians, double targetAngleRadians, ProfiledPIDController thetaController) {
    // Get current error to setpoint.
    currentHeadingRadians = Helpers.modRadians(currentHeadingRadians);
    targetAngleRadians = Helpers.modRadians(targetAngleRadians);
    double error = targetAngleRadians - currentHeadingRadians;
    // Apply PID controller to error and calculate value. Negate to get expected direction.
    return -thetaController.calculate(error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble(), 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.m_thetaController.atSetpoint();
    //return Math.abs(Helpers.modDegrees(m_targetAngle.getDegrees()) - Helpers.modDegrees(m_drive.getHeadingDegrees())) <= DriveAutoConstants.kRobotControllerTolerance.getRotation().getDegrees();
  }
}
