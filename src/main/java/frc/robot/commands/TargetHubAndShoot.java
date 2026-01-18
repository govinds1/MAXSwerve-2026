package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Helpers;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionTargeting;
import frc.robot.subsystems.VisionTargeting.ShootingInfo;

// Simply utility command for Autos to drive robot in a straight line without worrying about trajectory control
public class TargetHubAndShoot extends Command {

    private DriveSubsystem m_drive;
    private VisionTargeting m_vision;
    private Shooter m_shooter;
    
    private double m_startTime;

    private enum State {
        kFindHub, kAimAtHub, kShoot
    } 
    private State m_state;
    private ShootingInfo m_shot;
    private Alliance m_alliance;

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
    m_state = State.kFindHub;
    var a = DriverStation.getAlliance();
    if (a.isPresent()) {
        m_alliance = a.get();
    } else {
        m_alliance = null;
    }
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    // TODO: Turn this state machine into a sequential command group!
    switch (m_state)
    {
        case kFindHub:
            // 1. Using vision, find Hub, rotating based off gyro angle and robot pose to find it.

            // Try to get a shot at the Hub.
            m_shot = m_vision.getHubAimInfo();
            if (m_shot != null)
            {
                // A shot is available, so move to aiming state.
                m_state = State.kAimAtHub;
                return;
            }
            // No shot is available, so try to find Hub.
            if (m_alliance == null) {
                // Rotate a chosen direction.
                m_drive.drive(0, 0, -0.2, true);
            } else {
                Pose2d currentPose = m_drive.getPose();
                Translation2d hubLocation = (m_alliance == Alliance.Blue) ? FieldConstants.kBlueHub : FieldConstants.kRedHub;
                // TODO: If we're in the neutral zone, exit command? Can't shoot because of net.
                double hyp = hubLocation.getDistance(currentPose.getTranslation());
                double ydist = hubLocation.getY() - currentPose.getY();
                Rotation2d theta = new Rotation2d(Math.asin(ydist/hyp));
                Rotation2d currentHeading = m_drive.getHeadingRotation();
                // Theta range will be -90 (directly left of hub) to 90 (directly right of hub).
                // If currentHeading is within 180 degrees CCW of theta, turn right. Otherwise, turn left.
                double thetaDelta = Helpers.modDegrees(theta.minus(currentHeading).getDegrees()); // Normalize difference in angle to 0-360 range for ease.
                if (thetaDelta < 180)
                {
                    // Turn right.
                    m_drive.drive(0, 0, -0.2, false);
                } else {
                    // Turn left.
                    m_drive.drive(0, 0, 0.2, false);
                }
                
                // TODO: Or use PID?
                Rotation2d desiredHeading = new Rotation2d(Helpers.modRadians(theta.getRadians()));
                ChassisSpeeds robotSpeeds = m_drive.m_robotDriveController.calculate(
                    currentPose,
                    new Pose2d(currentPose.getTranslation(), desiredHeading),
                    0,
                    desiredHeading
                );
                m_drive.driveRobotRelative(robotSpeeds);
            }
            break;
        case kAimAtHub:
            // 2. Once can see Hub, get to a close enough distance to score (minimal move time) while rotating to aim at the Hub center
                // Optionally, if variable RPM doesn't work, then we need a good distance range where we know X RPM will score.
                // Once within that distance range, shoot at X RPM.
            // Get updated shot info.
            m_shot = m_vision.getHubAimInfo();
            if (m_shot == null) {
                // TODO: Maybe if this happens, we try again next iteration? Or we just skip this state and try to shoot.
                m_state = State.kFindHub;
                return;
            }

            
            break;
        case kShoot:
            // 3. Once aimed at Hub center, spin at RPM to shoot based off distance to center.
            break;
    }

    
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