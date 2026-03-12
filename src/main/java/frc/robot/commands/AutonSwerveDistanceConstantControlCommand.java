package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveAutoConstants;
import frc.robot.subsystems.DriveSubsystem;

// Simply utility command for Autos to drive robot in a straight line without worrying about trajectory control
public class AutonSwerveDistanceConstantControlCommand extends Command {

    private DriveSubsystem m_drive;

    private Pose2d m_startingPose;
    private Translation2d m_desiredTranslationDelta; // desired pose relative to starting pose. Example -> (2, -3) means we move 2 meters forward, 3 meters right
    private Rotation2d m_desiredRotation; // desired angle (field relative) at the end of driving. Example -> 0 means we face opponent's driver station.
    private boolean m_halfSpeed;

    private double m_powerMax = 0.7;
    private double m_powerMin = 0.1;
    private double m_rampDownDistMeters = 1.0;

    private boolean m_finished = false;
    

public AutonSwerveDistanceConstantControlCommand(DriveSubsystem subsystem, Translation2d desiredTranslationDelta, Rotation2d desiredRotation, boolean halfSpeed) {
    m_drive = subsystem;
    m_desiredTranslationDelta = desiredTranslationDelta;
    m_desiredRotation = desiredRotation;
    m_halfSpeed = halfSpeed; 
    addRequirements(m_drive);
}

public AutonSwerveDistanceConstantControlCommand(DriveSubsystem subsystem, Translation2d desiredTranslationDelta, Rotation2d desiredRotation) {
    this(subsystem, desiredTranslationDelta, desiredRotation, false);
}


// Called when the command is initially scheduled.
@Override
public void initialize() {
    m_startingPose = m_drive.getPose();
    m_finished = false;
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    // Get current pose.
    Pose2d currentPose = m_drive.getPose().relativeTo(m_startingPose);

    double translationXSpeed = 0;
    double translationYSpeed = 0;
    double rotationSpeed = 0;

    double errorX = m_desiredTranslationDelta.getX() - currentPose.getX();
    double errorY = m_desiredTranslationDelta.getY() - currentPose.getY();

    double maxPowerAdjusted = m_powerMax * ((m_halfSpeed) ? 0.5 : 1);

    translationXSpeed = calcSpeedRamp(errorX, DriveAutoConstants.kRobotControllerTolerance.getX(), maxPowerAdjusted);
    translationYSpeed = calcSpeedRamp(errorY, DriveAutoConstants.kRobotControllerTolerance.getY(), maxPowerAdjusted);

    rotationSpeed = TurnToAngle.getRotSpeed(currentPose.getRotation().getRadians(), m_desiredRotation.getRadians(), m_drive.m_thetaController);
    if (translationXSpeed == 0 && translationYSpeed == 0 && m_drive.m_thetaController.atSetpoint()) {
        m_finished = true;
    }

    // Apply output to drive.
    m_drive.drive(translationXSpeed, translationYSpeed, rotationSpeed, true);
}

public double calcSpeedRamp(double error, double tolerance, double maxPower) {
    double errorAbs = Math.abs(error);
    if (errorAbs > m_rampDownDistMeters) {
        // Set speed to max speed.
        return Math.signum(error) * maxPower;
    } else if (errorAbs > tolerance) {
        // Mimic PID - ramp down to min speed as error gets close to tolerance.
        double distRatio = MathUtil.clamp(errorAbs / m_rampDownDistMeters, 0, 1);
        double power = MathUtil.interpolate(m_powerMin, maxPower, distRatio);
        return Math.signum(error) * power;
    } else {
        // Within tolerance, so set to 0.
        return 0;
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
    return m_finished;
}

@Override
public boolean runsWhenDisabled() {
        return false;
    }
}