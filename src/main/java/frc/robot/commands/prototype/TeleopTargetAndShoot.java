// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.prototype;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.controllers.DriverController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionTargeting;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleopTargetAndShoot extends ParallelCommandGroup {
  /** Creates a new DriveTargetAndShoot. */
  public TeleopTargetAndShoot(DriveSubsystem drive, VisionTargeting vision, ShooterSubsystem shooter, DriverController controller) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    AimAtHubWhileDriving aimCommand = new AimAtHubWhileDriving(drive, vision, controller);
    Command shootWhenAtHub = new SequentialCommandGroup(
      // TODO: Add command that runs shooter while we're aimed. Replace idle command.
      Commands.idle().onlyWhile(aimCommand::isAimed)
    );
    addCommands(
      aimCommand,
      new RepeatCommand(shootWhenAtHub)
    );
  }
}
