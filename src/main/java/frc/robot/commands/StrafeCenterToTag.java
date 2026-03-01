// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionTargeting;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StrafeCenterToTag extends Command {
  private final DriveSubsystem m_drive;
  private final VisionTargeting m_vision; 

  private PIDController m_alignPID = new PIDController(0.01, 0, 0); // TODO: Tuned to limelight Tx values.

  /** Creates a new AlignToTarget. */
  // This should be used for aligning to tower, trench, and depot. Not for Hub aiming.
  public StrafeCenterToTag(DriveSubsystem drive, VisionTargeting vision) {
    m_drive = drive;
    m_vision = vision;

    m_alignPID.setTolerance(0.3); // TODO: Adjust.

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationY = m_alignPID.calculate(m_vision.getTx(), 0.0);
    m_drive.drive(0, translationY, 0, false);
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
    return m_alignPID.atSetpoint();
  }
}
