// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Helpers;
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
        .finallyDo(() -> robotDrive.stop());
    }

    public static Command DriveForDistance(DriveSubsystem robotDrive, Translation2d translationDelta, Rotation2d desiredRotation) {
        // Drives the robot based off the given field relative translationDelta and desiredRotation.
        // Uses a step function to determine speed along each translation axis and for rotation, starts at a higher speed and moves to lower speed when closer.
        // Example: DriveForDistance(robotDrive, new Translation2d(1.5, -3), Rotation2d.fromDegrees(90));
        //     - Drives the robot 1.5 meters FORWARD, 3 meters to the RIGHT, and rotates robot to be facing the left side wall (90 degrees CCW).

        // Define end conditions for each dimension.
        Pose2d startingPose = robotDrive.getPose();
        Translation2d directionVector = new Translation2d(Math.signum(translationDelta.getX()), Math.signum(translationDelta.getY()));
        BooleanSupplier isAtDistanceX = () -> (Math.abs(robotDrive.getPose().minus(startingPose).getX() - translationDelta.getX()) < 0.05);
        BooleanSupplier isAtDistanceY = () -> (Math.abs(robotDrive.getPose().minus(startingPose).getY() - translationDelta.getY()) < 0.05);
        BooleanSupplier isAtDistanceRot = () -> (Math.abs(robotDrive.getHeadingDegrees() - desiredRotation.getDegrees()) < 0.5);
        BooleanSupplier isAtDistance = () -> isAtDistanceX.getAsBoolean() && isAtDistanceY.getAsBoolean()&& isAtDistanceRot.getAsBoolean();

        // Define value suppliers for each dimension.
        double slowSpeed = 0.2;
        double highSpeed = 0.4;
        DoubleSupplier xSupplier = () -> {
            if (isAtDistanceX.getAsBoolean()) return 0;
            return directionVector.getX() * ((robotDrive.getPose().minus(startingPose).getX() < 0.5) ? slowSpeed : highSpeed);
        };
        DoubleSupplier ySupplier = () -> {
            if (isAtDistanceY.getAsBoolean()) return 0;
            return directionVector.getY() * ((robotDrive.getPose().minus(startingPose).getY() < 0.5) ? slowSpeed : highSpeed);
        };
        double slowRot = 0.15;
        double highRot = 0.3;
        DoubleSupplier rotSupplier = () -> {
            if (isAtDistanceRot.getAsBoolean()) return 0;
            double degreesToTurn = Helpers.quickestRot(robotDrive.getHeadingRotation(), desiredRotation).getDegrees();
            return Math.signum(degreesToTurn) * ((Math.abs(degreesToTurn) < 10) ?  slowRot : highRot);
        };

        // Create command to drive with suppliers and end conditions.
        return Commands.run(() -> robotDrive.drive(xSupplier, ySupplier, rotSupplier), robotDrive)
        .until(isAtDistance)
        .withTimeout(Math.max(2.5, translationDelta.getNorm() + (desiredRotation.getRadians() / 2.0)))
        .finallyDo(() -> robotDrive.stop());
    }

    public static Command DriveForDistance(DriveSubsystem robotDrive, Translation2d translationDelta) {
        return DriveForDistance(robotDrive, translationDelta, robotDrive.getHeadingRotation());
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
