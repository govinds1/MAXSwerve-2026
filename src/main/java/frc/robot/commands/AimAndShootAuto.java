package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveAutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionTargeting;
import frc.robot.subsystems.VisionTargeting.ShootingInfo;

// Simply utility command for Autos to drive robot in a straight line without worrying about trajectory control
public class AimAndShootAuto extends Command {

    private DriveSubsystem m_drive;
    private VisionTargeting m_vision;
    private ShooterSubsystem m_shooter;

    private ShootingInfo m_shot = null;

    private double m_startTime;
    private double m_shotStartTime;
    private boolean m_isFinished;
    private boolean m_isAimed;


// This command assumes full control of the chassis. It will get the robot into a proper pose to shoot and end ASAP.
public AimAndShootAuto(DriveSubsystem drive, VisionTargeting vision, ShooterSubsystem shooter) {
    m_drive = drive;
    m_vision = vision;
    m_shooter = shooter;
    addRequirements(m_drive, m_vision, m_shooter);
}


// Called when the command is initially scheduled.
@Override
public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
    m_isFinished = false;
    m_isAimed = false;
    // Stop drivetrain.
    m_drive.stop();
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    if (!m_isAimed) {
        // Get updated shot info.
        if (m_shot == null) {
            m_shot = m_vision.getHubAimInfo(m_drive.getPose(), m_drive.getRobotRelativeSpeeds());
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

        // After calling calculate, check if close enough to reference or if max aim time has elapsed.
        if (m_drive.m_robotDriveController.atReference() || ((Timer.getFPGATimestamp() - m_startTime) > DriveAutoConstants.kAimAtHubMaxTime)) {
            // Aiming is done.
            m_isAimed = true;
            m_shotStartTime = Timer.getFPGATimestamp();
            m_shooter.runShooterRPM(m_shot.shootingRPM);
            m_shooter.runFeeder(m_startTime);
        }
    } else {
        m_shooter.runShooterRPM(m_shot.shootingRPM);

        double shotTimeElapsed = Timer.getFPGATimestamp() - m_shotStartTime;
        // Run feeder if flywheel is at speed. TODO: Could use encoder to determine velocity - atSpeed() function.
        if (shotTimeElapsed > ShooterConstants.kShooterSpinUpTime) {
            m_shooter.runFeeder(ShooterConstants.kFeederPower);
        }
        // End command if we have shot for enough time. TODO: Better way to know we have no more fuel?
        if (shotTimeElapsed > ShooterConstants.kMaxShootTime) {
            m_isFinished = true;
        }
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
    return m_isFinished;
}

@Override
public boolean runsWhenDisabled() {
        return false;
    }
}