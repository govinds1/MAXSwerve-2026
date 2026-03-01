// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Helpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionTargeting;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToTarget extends SequentialCommandGroup {
  Rotation2d targetAngle;

  /** Creates a new AlignToTarget. */
  public AlignToTarget(DriveSubsystem drive, VisionTargeting vision) {
    // Targets will be facing or away from driver station, so find whether we are closer to 0/360 or 180.
    double offsetTo180 = 180 - Helpers.modDegrees(drive.getHeadingDegrees());
    if (Math.abs(offsetTo180) < 90) {
      // Closer to 180.
      targetAngle = Rotation2d.fromDegrees(180);
    } else {
      // Closer to 0/360.
      targetAngle = Rotation2d.fromDegrees(0);
    }

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TurnToAngle(drive, targetAngle, () -> 0, () -> 0).withTimeout(1.5),
      new StrafeCenterToTag(drive, vision).withTimeout(1.5)
    );
  }
}
