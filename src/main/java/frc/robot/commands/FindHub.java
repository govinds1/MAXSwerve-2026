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
import frc.robot.Constants.DriveAutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionTargeting;
import frc.robot.subsystems.VisionTargeting.ShootingInfo;

// Simply utility command for Autos to drive robot in a straight line without worrying about trajectory control
public class FindHub extends Command {

    private DriveSubsystem m_drive;
    private VisionTargeting m_vision;

    private ShootingInfo m_shot;
    private Alliance m_alliance;

    private double m_startTime;
    private boolean m_isFinished;

public FindHub(DriveSubsystem drive, VisionTargeting vision) {
    m_drive = drive;
    m_vision = vision;
    addRequirements(m_drive, m_vision);
}


// Called when the command is initially scheduled.
@Override
public void initialize() {
    // Get current alliance, if available.
    var a = DriverStation.getAlliance();
    if (a.isPresent()) {
        m_alliance = a.get();
    } else {
        m_alliance = null;
    }

    m_startTime = Timer.getFPGATimestamp();
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    //Using vision, find Hub, rotating based off gyro angle and estimated robot pose to find it.

    // Try to get a shot at the Hub.
    m_shot = m_vision.getHubAimInfo();
    if (m_shot != null)
    {
        // A shot is available! No need to do anything else.
        m_isFinished = true;
        return;
    }
    // No shot is available, rotate to try to find Hub.
    if (m_alliance == null) {
        // Rotate a chosen direction.
        m_drive.drive(0, 0, -0.2, true);
    } else {
        // TODO: Move to a closer spot while rotating, if necessary.

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
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    // End the command if too much time has passed, or we were (un)successful in finding the Hub.
    return m_isFinished || (Timer.getFPGATimestamp() - m_startTime) > DriveAutoConstants.kFindHubMaxTime;
}

@Override
public boolean runsWhenDisabled() {
        return false;

    }
}