// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Helpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionTargeting;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTarget extends Command {
  private final DriveSubsystem m_drive;
  private final VisionTargeting m_vision; 

  private double m_targetAngleDegrees; // Will either be 0 or 180 depending on target.
  private int m_state; // 0: Rotate, 1: Align, 2: Done
  private double m_startTime;

  private final PIDController m_alignPID = new PIDController(0.01, 0.0, 0);
  private final PIDController m_rotationPID = new PIDController(0.04, 0.0, 0.05);

  /** Creates a new AlignToTarget. */
  // This should be used for aligning to tower, trench, and depot. Not for Hub aiming.
  public AlignToTarget(DriveSubsystem drive, VisionTargeting vision) {
    m_drive = drive;
    m_vision = vision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Assume robot is already more or less facing the target (in order to see April Tag).
    // Rotate robot to 0, 180, or 360 degrees, depending on which is closer.
    double offsetTo180 = 180.0 - Helpers.modDegrees(m_drive.getHeadingDegrees());
    if (Math.abs(offsetTo180) <= 90) {
      m_targetAngleDegrees = 180.0;
    } else {
      if (offsetTo180 < 0) {
        m_targetAngleDegrees = 360.0;
      } else {
        m_targetAngleDegrees = 0.0;
      }
    }

    m_state = 0;
    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End if aligning is complete, or if timeout reached.
    return m_state == 2 || ((Timer.getFPGATimestamp() - m_startTime) > 6.0);
  }
}
