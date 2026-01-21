package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveAutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionTargeting;
import frc.robot.subsystems.VisionTargeting.ShootingInfo;

// Simply utility command for Autos to drive robot in a straight line without worrying about trajectory control
public class AimAtHub extends Command {

    private DriveSubsystem m_drive;
    private VisionTargeting m_vision;

    private ShootingInfo m_shot;

    private double m_startTime;
    private boolean m_isFinished;


public AimAtHub(DriveSubsystem drive, VisionTargeting vision) {
    m_drive = drive;
    m_vision = vision;
    addRequirements(m_drive, m_vision);
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
    // Get to a close enough distance to score (minimal move time) while rotating to aim at the Hub center.
        // Optionally, if variable RPM doesn't work, then we need a good distance range where we know X RPM will score.
        // Once within that distance range, shoot at X RPM.
    
    // Get updated shot info.
    if (m_shot == null) {
        m_shot = m_vision.getHubAimInfo();
        if (m_shot == null) {
            // Abandon ship! No shot exists.
            m_isFinished = true;
            return;
        }
    }

    // Calculate closed loop control feedback based on setpoint and current pose estimate.
    Pose2d currentPose = m_drive.getPose();
    ChassisSpeeds robotSpeeds = m_drive.m_robotDriveController.calculate(
        currentPose,
        m_shot.pose,
        0,
        m_shot.pose.getRotation()
    );
    // Set calculated robot relative speeds.
    m_drive.driveRobotRelative(robotSpeeds);

    // After calling calculate, check if close enough to reference.
    if (m_drive.m_robotDriveController.atReference()) {
        // Aiming is done.
        m_isFinished = true;
    }
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    //  too much time has passed, then end the command.
    return m_isFinished || (Timer.getFPGATimestamp() - m_startTime) > DriveAutoConstants.kAimAtHubMaxTime;
}

@Override
public boolean runsWhenDisabled() {
        return false;
    }
}