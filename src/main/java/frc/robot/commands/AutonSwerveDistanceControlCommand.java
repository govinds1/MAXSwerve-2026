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
    private double startTime;
    

public AutonSwerveDistanceControlCommand(DriveSubsystem subsystem, Translation2d desiredTranslationDelta, Rotation2d desiredRotation) {
    m_drive = subsystem;
    m_desiredTranslationDelta = desiredTranslationDelta;
    m_desiredRotation = desiredRotation;
    addRequirements(m_drive);
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
    ChassisSpeeds newSpeeds = m_drive.m_robotDriveController.calculate(currentPose, new Pose2d(m_desiredTranslationDelta, m_desiredRotation), 0, m_desiredRotation);
    // Apply output to drive.
    m_drive.driveRobotRelative(newSpeeds);
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    double maxAllowedTime = Math.max(Math.sqrt(m_desiredTranslationDelta.getSquaredNorm()), 2.0);
    return (Timer.getFPGATimestamp() - startTime) > maxAllowedTime;
}

@Override
public boolean runsWhenDisabled() {
        return false;

    }
}