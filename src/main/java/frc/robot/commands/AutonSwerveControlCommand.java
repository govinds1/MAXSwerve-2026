package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

// Simply utility command for Autos to drive robot in a straight line without worrying about trajectory control
public class AutonSwerveControlCommand extends Command {

    private DriveSubsystem m_drive;
    private double m_forwardSpeed;
    private double m_rightSpeed;
    private double m_time;
    private double m_omega;
    private boolean m_fieldRel;
    private double startTime;
    

public AutonSwerveControlCommand(DriveSubsystem subsystem, double forward, double right, double omega, double time, boolean fieldRel) {
    m_drive = subsystem;
    // remove add negative signs below if robot is moving the unintended way
    m_forwardSpeed = forward;  //2/27 changed from negative to positive to have robot move backwards
    m_rightSpeed = -right;
    m_omega = omega;
    m_time = time;
    m_fieldRel = fieldRel;
    addRequirements(m_drive);
}


// Called when the command is initially scheduled.
@Override
public void initialize() {
    startTime = Timer.getFPGATimestamp();
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
     m_drive.drive(m_forwardSpeed, m_rightSpeed, m_omega, m_fieldRel);
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime) > m_time;
}

@Override
public boolean runsWhenDisabled() {
        return false;

    }
}