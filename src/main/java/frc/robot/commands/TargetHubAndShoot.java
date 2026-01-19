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
    
    private boolean m_isFinished;
    private double m_shootStartTime;

    private enum TargetHubAndShootState {
        kFindHub, kAimAtHub, kShootStart, kShootWait, kEnd
    } 
    private TargetHubAndShootState m_state;
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
    m_isFinished = false;
    m_state = TargetHubAndShootState.kFindHub;
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
                m_state = TargetHubAndShootState.kAimAtHub;
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

                // Use PID for closed loop control.
                Rotation2d desiredHeading = new Rotation2d(Helpers.modRadians(theta.getRadians()));
                ChassisSpeeds robotSpeeds = m_drive.m_robotDriveController.calculate(
                    currentPose,
                    new Pose2d(currentPose.getTranslation(), desiredHeading),
                    0,
                    desiredHeading
                );
                m_drive.driveRobotRelative(robotSpeeds);

                
                /*
                // Open loop control.
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
                */
            }
            break;
        case kAimAtHub:
            // 2. Once can see Hub, get to a close enough distance to score (minimal move time) while rotating to aim at the Hub center
                // Optionally, if variable RPM doesn't work, then we need a good distance range where we know X RPM will score.
                // Once within that distance range, shoot at X RPM.
            // Get updated shot info.
            if (m_shot == null) {
                // Try to aim again.
                m_shot = m_vision.getHubAimInfo();
                if (m_shot == null) {
                    // TODO: Should we just abandon ship and move to shoot?
                    m_state = TargetHubAndShootState.kFindHub;
                    return;
                }
            }

            Pose2d currentPose = m_drive.getPose();
            ChassisSpeeds robotSpeeds = m_drive.m_robotDriveController.calculate(
                currentPose,
                m_shot.pose,
                0,
                m_shot.pose.getRotation()
            );
            m_drive.driveRobotRelative(robotSpeeds);

            // After calling calculate
            if (m_drive.m_robotDriveController.atReference()) {
                m_state = TargetHubAndShootState.kShootStart;
            }
            break;
        case kShootStart:
            // 3. Once aimed at Hub center, spin at RPM to shoot based off distance to center.
            m_shooter.run(m_shot.shootingRPM);
            m_shootStartTime = Timer.getFPGATimestamp();
            m_state = TargetHubAndShootState.kShootWait;
            break;
        case kShootWait:
            // 4. Wait for some time to finish shooting. 
            // TODO: If we have a sensor in the hopper, then use it to know when we have no more balls
            if (Timer.getFPGATimestamp() - m_shootStartTime > ShooterConstants.kMaxShootTime) {
                m_state = TargetHubAndShootState.kEnd;
            }
            break;
        case kEnd:
            // 5. Done!
            m_isFinished = true;
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
    return m_isFinished;
}

@Override
public boolean runsWhenDisabled() {
        return false;

    }
}