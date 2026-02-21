// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveAutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionTargeting;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimClosedLoop extends Command {

  private final DriveSubsystem m_drive;
  private final ShooterSubsystem m_shooter;
  private final VisionTargeting m_vision;
  
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;

  private final PIDController m_aimPID = new PIDController(DriveAutoConstants.kPThetaController, 0.0, 0.005); 

  public AimClosedLoop(DriveSubsystem drive, ShooterSubsystem shooter, VisionTargeting vision, 
                          DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    m_drive = drive;
    m_shooter = shooter;
    m_vision = vision;
    m_translationXSupplier = xSupplier;
    m_translationYSupplier = ySupplier;

    addRequirements(m_drive, m_shooter);
  }

  @Override
  public void execute() {
      double translationX = m_translationXSupplier.getAsDouble();
      double translationY = m_translationYSupplier.getAsDouble();
      
      double rotationSpeed = 0.0;
      double targetRpm = 1500.0;

      if (m_vision.hasTarget()) {
          
          rotationSpeed = m_aimPID.calculate(m_vision.getTx(), 0.0);
          
          double distance = m_vision.getDistanceToTargetMeters();
          targetRpm = ShooterSubsystem.calculateRPMForDistanceToHUB(distance);
      }

      // Aim Robot
      m_drive.drive(translationX, translationY, rotationSpeed, true); 

      // Rev the shooter
      m_shooter.runShooterRPM(targetRpm);
  }

  @Override
  public void end(boolean interrupted) {
      // Stop the shooter when the button is released, or return to an idle RPM
      m_shooter.stopShooter();
  }
}
