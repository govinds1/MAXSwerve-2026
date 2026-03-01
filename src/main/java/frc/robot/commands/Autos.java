// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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

    public static String[] autoNames = {"DriveBackAndShoot", "Shoot", "Outpost"};

    public static Command getSelectedAuto(String selectedAutoName, DriveSubsystem robotDrive, ShooterSubsystem shooter, 
            VisionTargeting vision, IntakeSubsystem intake) {
        Command command = null;

        switch(selectedAutoName) {
            case "DriveBackAndShoot":
            command = Autos.driveBackAndShoot(robotDrive, shooter, vision, intake);
            break;
            case "Shoot":
            command = new AimClosedLoop(robotDrive, shooter, vision, () -> 0, () -> 0, () -> 0).withTimeout(5.0);
            break;
            case "Outpost":
            command = Autos.outpostStart(robotDrive, shooter, vision, intake);
            break;
        }
        
        return command;
    }

    public static Command driveBackAndShoot(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake) {
        return new SequentialCommandGroup(
            new AutonSwerveControlCommand(robotDrive, -0.2, 0, -0.1, 3, true),
            intake.extendAuto(),
            new AimClosedLoop(robotDrive, shooter, vision, () -> 0, () -> 0, () -> 0).withTimeout(3.0)
        );
    }

    public static Command outpostStart(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake) {
        return new SequentialCommandGroup(
            // TODO:
            // 1. Aim and shoot at Hub.
            // 2. Drive to outpost.
            // 3. Align to outpost (or trench? if feeding from outpost from behind). TODO: Are trench and outpost lined up?
            // 4. Wait for fuel to be dumped.
            // 5. Aim and shoot at Hub.
            new AimClosedLoop(robotDrive, shooter, vision, () -> 0, () -> 0, () -> 0).withTimeout(3.0),
            new AutonSwerveControlCommand(robotDrive, -0.5, 0, 0, 2, true),
            new AlignToTarget(robotDrive, vision),
            new WaitCommand(1.5),
            new AutonSwerveControlCommand(robotDrive, 0.3, 0.3, 0.1, 1.5, true),
            new AimClosedLoop(robotDrive, shooter, vision, () -> 0, () -> 0, () -> 0).withTimeout(3.0)
        );
    }
}
