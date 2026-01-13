package frc.robot.commands;

import frc.robot.Constants.DriveAutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;


public class AutoTrajectory extends Command{

    // EXAMPLE TRAJECTORY FROM 2025
    public static Command leaveTrajectory(DriveSubsystem robotDrive) {
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    DriveAutoConstants.kMaxSpeedMetersPerSecond,
                    DriveAutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);
    
         // Generate trajectory (moves forward meters)
        Trajectory leaveTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(2.2, 0, new Rotation2d(0)),            //tested at 1.73 calculated to be 1.4 with back wheel on starting line
            config
        );
    
        var thetaController =
            new ProfiledPIDController(
                DriveAutoConstants.kPThetaController, 0, 0, DriveAutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                leaveTrajectory,
                robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,
    
                // Position controllers
                new PIDController(DriveAutoConstants.kPXController, 0, 0),
                new PIDController(DriveAutoConstants.kPYController, 0, 0),
                thetaController,
                robotDrive::setModuleStates,
                robotDrive);
    
        // Reset odometry to the starting pose of the trajectory.
        robotDrive.resetOdometry(leaveTrajectory.getInitialPose());
    
        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0, false));
    }
}
