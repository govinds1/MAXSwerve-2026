// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionTargeting;

public final class Autos {
    // TODO: USING AUTOBUILDER! Any simple/non-path following auto commands can be added here, and you can still use AutoBuilder.

    /** Example static factory for an autonomous command. */
    public static Command exampleAuto() {
    return Commands.none();
    }

    private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
    }

    public static String[] autoNames = {"Leave Trajectory"};

    public static Command getSelectedAuto(String selectedAutoName, DriveSubsystem robotDrive, ShooterSubsystem shooter, 
            VisionTargeting vision, IntakeSubsystem intake) {
        Command command = null;

        switch(selectedAutoName) {
            case "Leave Trajectory":
            command = AutoTrajectory.leaveTrajectory(robotDrive);
            break;
            case "DriveBackAimAndShoot":
            command = Autos.driveBackAimShoot(robotDrive, shooter, vision);
            break;
        }
        
        return command;
    }

    public static Command driveBackAimShoot(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision) {
        return new SequentialCommandGroup(
            new AutonSwerveControlCommand(robotDrive, -0.2, 0, -0.1, 3, true),
            new AimClosedLoop(robotDrive, shooter, vision, () -> 0, () -> 0, () -> 0).withTimeout(5.0)
        );
    }
}
