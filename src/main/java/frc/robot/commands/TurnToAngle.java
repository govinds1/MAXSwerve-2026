// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Helpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionTargeting;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToAngle extends Command {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;

  private Rotation2d m_targetAngle;
  private double m_startTime;

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
    // Get current error to setpoint.
    double currentHeadingRadians = Helpers.modRadians(m_drive.getHeadingRotation().getRadians());
    double error = m_targetAngle.getRadians() - currentHeadingRadians;
    // Apply PID controller to error and calculate value.
    double rotSpeed = m_drive.m_thetaController.calculate(error);
    // Apply rotational output to drive, along with strafe inputs from supplier.
    m_drive.drive(m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble(), rotSpeed, true);
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
  }
}
