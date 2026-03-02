package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Helpers;
import frc.robot.subsystems.DriveSubsystem;

// Simply utility command for Autos to drive robot in a straight line without worrying about trajectory control
public class AutonSwerveDistanceControlCommand extends Command {

    private DriveSubsystem m_drive;

    private Pose2d m_startingPose;
    private Pose2d m_desiredPoseDelta; // desired pose relative to starting pose. Example -> (2, -3, Math.PI) means we move 2 meters forward, 3 meters right, 90 degree turn CCW
    private double startTime;
    

public AutonSwerveDistanceControlCommand(DriveSubsystem subsystem, Pose2d desiredPoseDelta) {
    m_drive = subsystem;
    m_desiredPoseDelta = desiredPoseDelta;
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
    ChassisSpeeds newSpeeds = m_drive.m_robotDriveController.calculate(currentPose, m_desiredPoseDelta, 0, m_desiredPoseDelta.getRotation());
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
    double maxAllowedTime = Math.max(Math.sqrt(m_desiredPoseDelta.getTranslation().getSquaredNorm()), 2.0);
    return (Timer.getFPGATimestamp() - startTime) > maxAllowedTime;
}

@Override
public boolean runsWhenDisabled() {
        return false;

    }
}