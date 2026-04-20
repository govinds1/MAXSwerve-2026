// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionTargeting;

public final class Autos {
    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static ArrayList<String> autoNames = new ArrayList<>(Arrays.asList("ShootStraight", "DoNothing"));

    public static HashMap<String, Command> PPAutos = new HashMap<>();

    public static Command getSelectedAuto(String selectedAutoName, DriveSubsystem robotDrive, ShooterSubsystem shooter, 
            VisionTargeting vision, IntakeSubsystem intake) {
        Command command = null;

        switch(selectedAutoName) {
            case "DoNothing":
            command = Commands.idle();
            break;
            case "ShootStraight":
            command = Autos.AimAndShootCommand(robotDrive, shooter, vision, intake);
            break;
            default:
            // Return PP auto, or none.
            command = PPAutos.getOrDefault(selectedAutoName, Commands.idle());
            break;
        }
        return command;
    }

    public static Command AimAndShootCommand(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake) {
        return AimAndShootCommand(robotDrive, shooter, vision, intake, ShooterConstants.kMaxShootTime);
    }

    public static Command AimAndShootCommand(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake, double timeout) {
        return Commands.race(
            new AimClosedLoop(robotDrive, shooter, vision, () -> 0, () -> 0, () -> 0),
            Commands.sequence(
                Commands.runOnce(() -> intake.runRollerRPM(), intake),
                intake.agitateAuto().withTimeout(timeout),
                Commands.runOnce(() -> intake.stopRoller(), intake),
                intake.retractAuto()
            )
        );
    }

    public static Command ShootNoAimCommand(ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake) {
        return ShootNoAimCommand(shooter, vision, intake, ShooterConstants.kMaxShootTime);
    }

    public static Command ShootNoAimCommand(ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake, double timeout) {
        Command intakeAgitationCommand = (timeout == -1) ? intake.agitateAuto() : intake.agitateAuto().withTimeout(timeout);
        return Commands.race(
            shooter.ShootStraightCommand(() -> vision.getDistanceToTargetMeters()),
            Commands.sequence(
                Commands.runOnce(() -> intake.runRollerRPM(), intake),
                intakeAgitationCommand,
                Commands.runOnce(() -> intake.stopRoller(), intake),
                intake.retractAuto()
            )
        );
    }

    public static Command DriveForSeconds(DriveSubsystem robotDrive, double forward, double left, double omega, double time) {
        return Commands.runOnce(() -> robotDrive.drive(forward, left, omega, true), robotDrive)
            .withTimeout(time)
            .finallyDo(() -> robotDrive.stop()
        );
    }

    public static Command RaiseClimber(ClimberSubsystem climber) {
        return Commands.runOnce(() -> climber.raiseHook(), climber)
            .until(() -> climber.isRaised())
            .withTimeout(ClimberConstants.kClimberUpTime)
            .finallyDo(() -> climber.stop()
        );
    }

    public static Command LowerClimber(ClimberSubsystem climber) {
        return Commands.runOnce(() -> climber.lowerHook(), climber)
            .until(() -> climber.isLowered())
            .withTimeout(ClimberConstants.kClimberDownTime)
            .finallyDo(() -> climber.stop()
        );
    }
}
