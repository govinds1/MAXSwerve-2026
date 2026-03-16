// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberCommand extends Command {

  private ClimberSubsystem m_climber;
  private boolean m_raise;

  /** Creates a new ClimberCommand. */
  public ClimberCommand(ClimberSubsystem climber, boolean raise) {
    m_climber = climber;
    m_raise = raise;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_raise) {
      m_climber.raiseHook();
    } else {
      m_climber.lowerHook();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_raise) {
      return m_climber.isRaised();
    } else {
      return m_climber.isLowered();
    }
  }
}
