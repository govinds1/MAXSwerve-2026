// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Helpers;
import frc.robot.Constants.FieldConstants;
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

    public static String[] autoNames = {"ShootStraight", "DoNothing",
        "ShootPreloads_StartRight", "ShootPreloads_StartLeft", "ShootPreloads_StartCenter",
        "ShootAndTrench_StartRight", "ShootAndTrench_StartLeft", 
        "TestMovement", "TestPathPlanner" //, "Test", "ShootAndOutpost_StartRight",
        //"QuickTrenchShoot_StartLeft", "QuickTrenchShoot_StartRight",
        //"ShootAndTrenchAndOutpost_StartRight"
    };
    
    enum StartSide {
        kLEFT, kCENTER, kRIGHT
    }
    private static StartSide m_startingSide = StartSide.kRIGHT;

    public static Command getSelectedAuto(String selectedAutoName, DriveSubsystem robotDrive, ShooterSubsystem shooter, 
            VisionTargeting vision, IntakeSubsystem intake) {
        Command command = null;

        if (selectedAutoName.contains("StartRight")) {
            m_startingSide = StartSide.kRIGHT;
        } else if (selectedAutoName.contains("StartLeft")) {
            m_startingSide = StartSide.kLEFT;
        } else if (selectedAutoName.contains("StartCenter")) {
            m_startingSide = StartSide.kCENTER;
        }

        switch(selectedAutoName) {
            case "DoNothing":
            command = Commands.idle();
            break;
            case "ShootStraight":
            command = Autos.AimAndShootCommand(robotDrive, shooter, vision, intake);
            break;
            case "Test":
            command = Commands.sequence(
                //new AutonSwerveDistanceControlCommand(robotDrive, new Translation2d(-1, -2.25), Rotation2d.fromDegrees(-30)), // TODO: Test with rotation after tuning translation.
                new AutonSwerveDistanceControlCommand(robotDrive, new Translation2d(-2, -2.3), Rotation2d.fromDegrees(0)),
                Commands.runOnce(() -> intake.runRoller(), intake),
                Commands.waitSeconds(0.35),
                new AutonSwerveDistanceControlCommand(robotDrive, new Translation2d(0.75, -0.75), Rotation2d.fromDegrees(0), true),
                Commands.waitSeconds(0.5),
                new AutonSwerveDistanceControlCommand(robotDrive, new Translation2d(-0.75, 0.75), Rotation2d.fromDegrees(-25)),
                Autos.AimAndShootCommand(robotDrive, shooter, vision, intake),
                new AutonSwerveDistanceControlCommand(robotDrive, new Translation2d(1.5, 2.3), Rotation2d.fromDegrees(0))
                //new TurnToAngle(robotDrive, Rotation2d.fromDegrees(-45), () -> 0, () -> 0),
                //new AimClosedLoop(robotDrive, shooter, vision, () -> 0, () -> 0, () -> 0).withTimeout(6.0)
                //new TurnToAngle(robotDrive, Rotation2d.fromDegrees(180), () -> 0, () -> 0),
                /*
                Commands.runOnce(() -> shooter.runShooterOpenLoop(0.2), shooter),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> shooter.stop(), shooter)
                */
            );
            break;
            case "TestMovement":
            command = Commands.sequence(
                //new AutonSwerveDistanceConstantControlCommand(robotDrive, new Translation2d(2, 1), Rotation2d.fromDegrees(45)),
                //new AutonSwerveDistanceTrapezoidalControlCommand(robotDrive, new Translation2d(-2, -1), Rotation2d.fromDegrees(135))
                new AutonSwerveDistanceControlCommand(robotDrive, new Translation2d(2,2), Rotation2d.fromDegrees(0)),
                new AutonSwerveDistanceControlCommand(robotDrive, new Translation2d(-2,-2), Rotation2d.fromDegrees(0))
            );
            break;
            case "TestPathPlanner":
            try {
                PathPlannerPath testPath = PathPlannerPath.fromPathFile("Test");
                command = AutoBuilder.followPath(testPath);
            } catch (Exception e) {
                DriverStation.reportError("PathPlanner follow path error: " + e.getMessage(), e.getStackTrace());
                return Commands.none();
            }
            break;
            case "ShootPreloads_StartRight":
            //command = Autos.turnAndShoot(robotDrive, shooter, vision, intake);
            command = Autos.turnAndShootTimeBased(robotDrive, shooter, vision, intake);
            break;
            case "ShootPreloads_StartLeft":
            //command = Autos.turnAndShoot(robotDrive, shooter, vision, intake);
            command = Autos.turnAndShootTimeBased(robotDrive, shooter, vision, intake);
            break;
            case "ShootPreloads_StartCenter":
            command = Autos.turnAndShootDriveControl(robotDrive, shooter, vision, intake);
            break;
            case "ShootAndOutpost_StartRight":
            command = Autos.shootAndOutpostTimeBased(robotDrive, shooter, vision, intake);
            break;
            case "ShootAndTrench_StartRight":
            //command = Autos.shootAndTrench(robotDrive, shooter, vision, intake);
            command = Autos.shootAndTrenchTimeBased(robotDrive, shooter, vision, intake);
            break;
            case "ShootAndTrench_StartLeft":
            //command = Autos.shootAndTrench(robotDrive, shooter, vision, intake);
            command = Autos.shootAndTrenchTimeBased(robotDrive, shooter, vision, intake);
            break;
            case "ShootAndTrenchAndOutpost_StartRight":
            command = Autos.shootAndTrenchAndOutpost(robotDrive, shooter, vision, intake);
            break;
            case "QuickTrenchShoot_StartLeft":
            command = Autos.quickTrenchAndShoot(robotDrive, shooter, vision, intake);
            break;
            case "QuickTrenchShoot_StartRight":
            command = Autos.quickTrenchAndShoot(robotDrive, shooter, vision, intake);
            break;
            default:
            command = Commands.idle();
            break;
        }
        
        return command;
    }

    public static Pose2d getStartingPose() {
        // TODO: 
        // When using PathPlanner, use this starting pose to set drive Pose (for estimator). 
        // Enable using different pose for different alliance, so we know starting rotation.
        switch (m_startingSide) {
            case kLEFT:
                return (Helpers.onRedAlliance()) ? FieldConstants.kRedLeftStart : FieldConstants.kBlueLeftStart;
                //return FieldConstants.kBlueLeftStart;
            case kCENTER:
                return (Helpers.onRedAlliance()) ? FieldConstants.kRedCenterStart : FieldConstants.kBlueCenterStart;
                //return FieldConstants.kBlueCenterStart;
            case kRIGHT:
                return (Helpers.onRedAlliance()) ? FieldConstants.kRedRightStart : FieldConstants.kBlueRightStart;
                //return FieldConstants.kBlueRightStart;
            default:
                return (Helpers.onRedAlliance()) ? FieldConstants.kRedRightStart : FieldConstants.kBlueRightStart;
                //return FieldConstants.kBlueRightStart;
        }
    }

    public static Pose2d getShootingPose() {
        // TODO: 
        // When using PathPlanner, use this starting pose to set drive Pose (for estimator). 
        // Enable using different pose for different alliance, so we know starting rotation.
        switch (m_startingSide) {
            case kLEFT:
                //return (Helpers.onRedAlliance()) ? FieldConstants.kRedLeftShootingPosition : FieldConstants.kBlueLeftShootingPosition;
                return FieldConstants.kBlueLeftShootingPosition;
            case kCENTER:
                //return (Helpers.onRedAlliance()) ? FieldConstants.kRedCenterShootingPosition : FieldConstants.kBlueCenterShootingPosition;
                return FieldConstants.kBlueCenterShootingPosition;
            case kRIGHT:
                //return (Helpers.onRedAlliance()) ? FieldConstants.kRedRightShootingPosition : FieldConstants.kBlueRightShootingPosition;
                return FieldConstants.kBlueRightShootingPosition;
            default:
                //return (Helpers.onRedAlliance()) ? FieldConstants.kRedRightShootingPosition : FieldConstants.kBlueRightShootingPosition;
                return FieldConstants.kBlueRightShootingPosition;
        }
    }

    public static Command AimAndShootCommand(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake) {
        return Commands.parallel(
            new AimClosedLoop(robotDrive, shooter, vision, () -> 0, () -> 0, () -> 0),
            Commands.repeatingSequence(
                intake.retractAuto(),
                intake.extendAuto()
            ),
            Commands.sequence(
                Commands.waitSeconds(2.0),
                Commands.runOnce(() -> intake.runRoller(), intake)
            )
        )
        .withTimeout(6.0) // TODO: Adjust this, can probably be quicker?
        .andThen(
            Commands.parallel(
                intake.extendAuto(),
                Commands.runOnce(() -> intake.stopRoller(), intake)
            )
        );
    }

    private static Command AimAndShootCommand_WithMovement(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake, Translation2d translationMovementVector, double time) {
        return Commands.parallel(
            new AimClosedLoopAdvanced(robotDrive, shooter, vision, () -> translationMovementVector.getX(), () -> translationMovementVector.getY(), () -> 0),
            Commands.repeatingSequence(
                intake.retractAuto(),
                intake.extendAuto()
            ),
            Commands.runOnce(() -> intake.runRoller(), intake)
        )
        .withTimeout(time)
        .andThen(
            Commands.parallel(
                intake.extendAuto(),
                Commands.runOnce(() -> intake.stopRoller(), intake)
            )
        );
    }

    private static Command AimAndShootCommand_WhileIntaking(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake, Translation2d translationMovementVector, double time) {
        return Commands.sequence(
            intake.extendAuto(),    
            Commands.parallel(
                new AimClosedLoopAdvanced(robotDrive, shooter, vision, () -> translationMovementVector.getX(), () -> translationMovementVector.getY(), () -> 0),
                Commands.runOnce(() -> intake.runRoller(), intake)
            )
            .withTimeout(time)
            .andThen(
                Commands.parallel(
                    intake.extendAuto(),
                    Commands.runOnce(() -> intake.stopRoller(), intake)
                )
            )
        );
    }

    public static Command turnAndShootTimeBased(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake) {
        // Assumes we are starting in line with trench and outpost.
        int rotDirection = (m_startingSide == StartSide.kRIGHT) ? 1 : -1;
        return new SequentialCommandGroup(
            // Rotate to roughly aim.
            new AutonSwerveTimeControlCommand(robotDrive, 0, 0, rotDirection * 0.35, 0.75, true),
            // Extend intake.
            intake.extendAuto(),
            // Shoot.
            Autos.AimAndShootCommand(robotDrive, shooter, vision, intake)
        );
    }

    public static Command turnAndShoot(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake) {
        // Assumes we are starting in line with trench and outpost.
        return new SequentialCommandGroup(
            // Turn to approx. face Hub and extend intake.
            Commands.parallel(
                //new TurnToAngle(robotDrive, getShootingPose().getRotation(), () -> 0, () -> 0),
                new AutonSwerveDistanceControlCommand(robotDrive, new Translation2d(0, 0), getShootingPose().getRotation()),
                intake.extendAuto()
            ).withTimeout(3), // 1.5
            // Shoot.
            Autos.AimAndShootCommand(robotDrive, shooter, vision, intake)
        );
    }

    public static Command turnAndShootDriveControl(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake) {
        // Assumes we are starting in line with trench and outpost.
        return new SequentialCommandGroup(
            // Turn to approx. face Hub and extend intake.
            Commands.parallel(
                new AutonSwerveDistanceControlCommand(robotDrive, getStartingPose().getTranslation().minus(getShootingPose().getTranslation()), getShootingPose().getRotation()),
                intake.extendAuto()
            ),
            // Shoot.
            Autos.AimAndShootCommand(robotDrive, shooter, vision, intake)
        );
    }

    public static Command shootAndOutpostTimeBased(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake) {
        // Assumes we are starting on right side, in line with trench and outpost.
        return new SequentialCommandGroup(
            // TODO:
            // 1. Aim and shoot at Hub.
            // 2. Drive to outpost.
            // 3. Align to outpost (or trench? if feeding from outpost from behind). TODO: Are trench and outpost lined up?
            // 4. Wait for fuel to be dumped.
            // 5. Aim and shoot at Hub.
            turnAndShootTimeBased(robotDrive, shooter, vision, intake),
            new AutonSwerveTimeControlCommand(robotDrive, -0.3, 0, 0, 2, true), // TODO: Update time parameter to get to outpost.
            //new AlignToTarget(robotDrive, vision),
            new WaitCommand(2.5), // TODO: Update how long we need to wait at outpost.
            new AutonSwerveTimeControlCommand(robotDrive, 0.3, 0, 0, 2, true), // TODO: Update time parameter to get back into shooting position.
            Autos.AimAndShootCommand(robotDrive, shooter, vision, intake)
        );
    }

    public static Command shootAndOutpost(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake) {
        // Assumes we are starting in line with trench and outpost.
        return new SequentialCommandGroup(
            // TODO:
            // 1. Aim and shoot at Hub.
            // 2. Drive to outpost.
            // 3. Align to outpost (or trench? if feeding from outpost from behind). TODO: Are trench and outpost lined up?
            // 4. Wait for fuel to be dumped.
            // 5. Aim and shoot at Hub.
            turnAndShoot(robotDrive, shooter, vision, intake),
            new AutonSwerveDistanceControlCommand(robotDrive, new Translation2d(-FieldConstants.kOutpostToStartLineMeters, 0), Rotation2d.fromDegrees(0)),
            new WaitCommand(2.5), // TODO: Update how long we need to wait at outpost.
            new AutonSwerveDistanceControlCommand(robotDrive, new Translation2d(FieldConstants.kOutpostToStartLineMeters, 0), getShootingPose().getRotation()),
            Autos.AimAndShootCommand(robotDrive, shooter, vision, intake)
        );
    }

    public static Command shootAndTrench(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake) {
        // Assumes we are starting in line with trench and outpost.
        return new SequentialCommandGroup(
            // 1. Aim and shoot at Hub.
            // 2. Drive thru trench until close to half line, turn to face fuel.
            // 3. Intake fuel, driving forward.
            // 4. Drive back to trench.
            // 5. Drive thru trench to shooting position.
            // 6. Aim and shoot at Hub.
            turnAndShoot(robotDrive, shooter, vision, intake),
            Autos.traverseTrenchGetFuel(robotDrive, shooter, vision, intake)
            //Autos.traverseTrenchGetFuelAndReturn(robotDrive, shooter, vision, intake),
            //turnAndShoot(robotDrive, shooter, vision, intake)
        );
    }

    private static Command shootAndTrenchTimeBased(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake) {
        // Assumes we are starting in line with trench and outpost.
        // FOR INTERMEDIATE USE ONLY!

        // If went through right trench, travel left (+) to refuel. Otherwise, travel right (-).
        int yDirection = (m_startingSide == StartSide.kRIGHT) ? 1 : -1;
        return new SequentialCommandGroup(
            // Aim and shoot.
            turnAndShootTimeBased(robotDrive, shooter, vision, intake),
            // Go to refuel.
            Commands.parallel(
                new AutonSwerveTimeControlCommand(robotDrive, 0.5, 0, 0, 2.0, true),
                intake.extendAuto()
            ),
            Commands.runOnce(() -> intake.runRollerRPM(), intake),
            new WaitCommand(0.5),
            new AutonSwerveTimeControlCommand(robotDrive, 0, yDirection * 0.2, 0, 2.0, true),
            Commands.runOnce(() -> robotDrive.stop())
            //new AutonSwerveDistanceControlCommand(robotDrive, new Translation2d(0, yDirection * 1.0), intakeHeading, 0, true)
        );
    }

    private static Command traverseTrenchGetFuelAndReturn(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake) {
        // Assumes we are starting in line with trench and outpost.
        // FOR INTERMEDIATE USE ONLY!

        // If went through right trench, travel left (+) to refuel. Otherwise, travel right (-).
        int yDirection = (m_startingSide == StartSide.kRIGHT) ? 1 : -1;
        Rotation2d intakeHeading = Rotation2d.fromDegrees(yDirection * 100);
        return new SequentialCommandGroup(
            // Go to refuel.
            Commands.parallel(
                new AutonSwerveDistanceControlCommand(robotDrive, new Translation2d(FieldConstants.kStartLineToOverCenterLineMeters, 0), intakeHeading),
                intake.extendAuto()
            ),
            Commands.parallel(
                new AutonSwerveDistanceControlCommand(robotDrive, new Translation2d(0, yDirection * FieldConstants.kEdgeToCenterFuelPickupMeters), intakeHeading, true),
                Commands.runOnce(() -> intake.runRollerRPM(), intake)
            ),
            Commands.waitSeconds(0.5),
            // Travel back.
            new AutonSwerveDistanceControlCommand(robotDrive, new Translation2d(0, -yDirection * FieldConstants.kEdgeToCenterFuelPickupMeters), getShootingPose().getRotation()),
            Commands.runOnce(() -> intake.stopRoller(), intake),
            new AutonSwerveDistanceControlCommand(robotDrive, new Translation2d(-FieldConstants.kStartLineToOverCenterLineMeters, 0), getShootingPose().getRotation(), true)
        );
    }

    private static Command traverseTrenchGetFuel(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake) {
        // Assumes we are starting in line with trench and outpost.
        // FOR INTERMEDIATE USE ONLY!

        // If went through right trench, travel left (+) to refuel. Otherwise, travel right (-).
        int yDirection = (m_startingSide == StartSide.kRIGHT) ? 1 : -1;
        Rotation2d intakeHeading = Rotation2d.fromDegrees(yDirection * 100);
        return new SequentialCommandGroup(
            // Go to refuel.
            Commands.parallel(
                new AutonSwerveDistanceControlCommand(robotDrive, new Translation2d(FieldConstants.kStartLineToOverCenterLineMeters, 0), intakeHeading),
                intake.extendAuto()
            ),
            Commands.parallel(
                new AutonSwerveDistanceControlCommand(robotDrive, new Translation2d(0, yDirection * FieldConstants.kEdgeToCenterFuelPickupMeters), intakeHeading, true),
                Commands.runOnce(() -> intake.runRollerRPM(), intake)
            ),
            Commands.runOnce(() -> robotDrive.stop())
            //new AutonSwerveDistanceControlCommand(robotDrive, new Translation2d(0, yDirection * 1.0), intakeHeading, 0, true)
        );
    }

    public static Command shootAndTrenchAndOutpost(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake) {
        // Assumes we are starting in line with trench and outpost.
        return new SequentialCommandGroup(
            // 1. Aim and shoot at Hub.
            // 2. Drive thru trench until close to half line, turn to face fuel.
            // 3. Intake fuel, driving forward.
            // 4. Drive back to trench.
            // 5. Drive thru trench to shooting position.
            // 6. Aim and shoot at Hub.
            // 7. Drive to outpost.
            // 8. Wait for fueling.
            // 9. Drive to shooting position.
            // 10. Aim and shoot.
            turnAndShoot(robotDrive, shooter, vision, intake),
            Autos.traverseTrenchGetFuelAndReturn(robotDrive, shooter, vision, intake),
            Autos.shootAndOutpost(robotDrive, shooter, vision, intake)
        );
    }

    public static Command quickTrenchAndShoot(DriveSubsystem robotDrive, ShooterSubsystem shooter, VisionTargeting vision, IntakeSubsystem intake) {
        // Assumes we are starting in line with trench and outpost.
        return new SequentialCommandGroup(
            // Works the same as shootAndTrench, but this auto avoids shooting first in order to try and beat other teams to the midfield first. 
            Autos.traverseTrenchGetFuelAndReturn(robotDrive, shooter, vision, intake),
            Autos.turnAndShoot(robotDrive, shooter, vision, intake)
        );
    }
}
