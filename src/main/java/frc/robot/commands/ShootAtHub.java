package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionTargeting;
import frc.robot.subsystems.VisionTargeting.ShootingInfo;

// Simply utility command for Autos to drive robot in a straight line without worrying about trajectory control
public class ShootAtHub extends Command {
    private ShooterSubsystem m_shooter;
    private VisionTargeting m_vision;

    private ShootingInfo m_shot;

    private double m_startTime;
    private boolean m_isFinished;


public ShootAtHub(ShooterSubsystem shooter, VisionTargeting vision) {
    m_shooter = shooter;
    m_vision = vision;
    addRequirements(m_shooter, m_vision);
}


// Called when the command is initially scheduled.
@Override
public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
    m_isFinished = false;
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    // Get updated shot info.
    if (m_shot == null) {
        // TODO: We need to know current pose/speeds to get this. Might have to combine with the other command.
        m_shot = m_vision.getHubAimInfo(new Pose2d(), new ChassisSpeeds());
        if (m_shot == null) {
            // Abandon ship! No shot exists.
            m_isFinished = true;
            return;
        }
    }

    // Spin at RPM to shoot based off distance to center.
    m_shooter.runShooterRPM(m_shot.shootingRPM);
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
    m_shooter.stop();
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    //  too much time has passed, then end the command.
    return m_isFinished || (Timer.getFPGATimestamp() - m_startTime) > ShooterConstants.kMaxShootTime;
}

@Override
public boolean runsWhenDisabled() {
        return false;
    }
}