package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

// Simply utility command for Autos to drive robot in a straight line without worrying about trajectory control
public class AutonSwerveDistanceControlCommand extends Command {

    private DriveSubsystem m_drive;

    private Pose2d m_startingPose;
    private Translation2d m_desiredTranslationDelta; // desired pose relative to starting pose. Example -> (2, -3) means we move 2 meters forward, 3 meters right
    private Rotation2d m_desiredRotation; // desired angle (field relative) at the end of driving. Example -> 0 means we face opponent's driver station.
    private double m_desiredEndVelocityMps; // desired linear velocity (mps) at the end of route. 
    private double startTime;
    private boolean m_halfSpeed;
    

public AutonSwerveDistanceControlCommand(DriveSubsystem subsystem, Translation2d desiredTranslationDelta, Rotation2d desiredRotation, double desiredEndVelocityMps, boolean halfSpeed) {
    m_drive = subsystem;
    m_desiredTranslationDelta = desiredTranslationDelta;
    m_desiredRotation = desiredRotation;
    m_desiredEndVelocityMps = desiredEndVelocityMps;
    m_halfSpeed = halfSpeed; 
    addRequirements(m_drive);
}

public AutonSwerveDistanceControlCommand(DriveSubsystem subsystem, Translation2d desiredTranslationDelta, Rotation2d desiredRotation, double desiredEndVelocityMps) {
    this(subsystem, desiredTranslationDelta, desiredRotation, desiredEndVelocityMps, false);
}

public AutonSwerveDistanceControlCommand(DriveSubsystem subsystem, Translation2d desiredTranslationDelta, Rotation2d desiredRotation) {
    this(subsystem, desiredTranslationDelta, desiredRotation, 0, false);
}


// Called when the command is initially scheduled.
@Override
public void initialize() {
    m_startingPose = m_drive.getPose();
    startTime = Timer.getFPGATimestamp();
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    // Get current pose.
    Pose2d currentPose = m_drive.getPose().relativeTo(m_startingPose);
    // Apply PID controllers to get output.
    ChassisSpeeds newSpeeds = m_drive.m_robotDriveController.calculate(currentPose, new Pose2d(m_desiredTranslationDelta, Rotation2d.fromDegrees(0)), 0, m_desiredRotation);
    if (m_halfSpeed) {
        if (Math.abs(newSpeeds.vxMetersPerSecond) > 1.5 || Math.abs(newSpeeds.vyMetersPerSecond) > 1.5) {
            newSpeeds = newSpeeds.times(0.5);
        }
    }
    // Apply output to drive.
    m_drive.driveRobotRelative(newSpeeds);
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
    //if (m_desiredEndVelocityMps == 0) {
        m_drive.drive(0, 0, 0, false);
    //}
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    double maxAllowedTime = Math.max(m_desiredTranslationDelta.getNorm(), 5.0); // TODO: For testing, set unlimited time.
    return ((Timer.getFPGATimestamp() - startTime) > maxAllowedTime) || m_drive.m_robotDriveController.atReference();
}

@Override
public boolean runsWhenDisabled() {
        return false;
    }
}