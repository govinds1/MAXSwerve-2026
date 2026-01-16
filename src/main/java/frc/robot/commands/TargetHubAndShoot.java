package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionTargeting;

// Simply utility command for Autos to drive robot in a straight line without worrying about trajectory control
public class TargetHubAndShoot extends Command {

    private DriveSubsystem m_drive;
    private VisionTargeting m_vision;
    private Shooter m_shooter;
    
    private double m_startTime;

public TargetHubAndShoot(DriveSubsystem subsystem, VisionTargeting vision, Shooter shooter) {
    m_drive = subsystem;
    m_vision = vision;
    m_shooter = shooter;
    addRequirements(m_drive, m_vision, m_shooter);
}


// Called when the command is initially scheduled.
@Override
public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
    m_shooter.stop();
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    return (Timer.getFPGATimestamp() - m_startTime) > ShooterConstants.kMaxAimTime;
}

@Override
public boolean runsWhenDisabled() {
        return false;

    }
}