// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
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
  private final DoubleSupplier m_rotSupplier;

  private DoubleSupplier m_visionOverrideSupplier;

  private final PIDController m_aimPID = new PIDController(0.03, 0.0, 0); // tuned for Limelight Tx values. (0.05, 0.0, 0.005)
  private double m_targetRpm;
  private boolean m_isAimed; 
  private double m_overrideStartTime = 0;
  private boolean m_noTarget = false;

  public AimClosedLoop(DriveSubsystem drive, ShooterSubsystem shooter, VisionTargeting vision, 
                          DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier visionOverrideSupplier) {
    this(drive, shooter, vision, xSupplier, ySupplier, null, visionOverrideSupplier);
  }

  public AimClosedLoop(DriveSubsystem drive, ShooterSubsystem shooter, VisionTargeting vision, 
                          DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier, DoubleSupplier visionOverrideSupplier) {
    m_drive = drive;
    m_shooter = shooter;
    m_vision = vision;
    m_translationXSupplier = xSupplier;
    m_translationYSupplier = ySupplier;
    m_rotSupplier = rotSupplier;
    m_visionOverrideSupplier = visionOverrideSupplier;
    m_isAimed = false;
    m_targetRpm = ShooterSubsystem.calculateRPMForDistanceToHUB(1.75);

    addRequirements(m_drive, m_shooter);

    SmartDashboard.putNumber("Subsystems/Vision/Auto/RotationSpeed", 0);
    SmartDashboard.putNumber("Subsystems/Vision/Auto/TargetRPM", 0);
    SmartDashboard.putBoolean("Subsystems/Vision/Auto/IsAimed", false);
  }

  @Override
  public void initialize() {
    m_isAimed = false;
    m_targetRpm = ShooterSubsystem.calculateRPMForDistanceToHUB(1.75);
    m_noTarget = false;
    m_overrideStartTime = 0;
  }

  @Override
  public void execute() {
      double translationX = 0.0;
      double translationY = 0.0;
      double rotationSpeed = 0.0;

      double visionOverride = m_visionOverrideSupplier.getAsDouble();
      if (visionOverride != 0) {
        if (visionOverride == 1) {
          // Shoot from Hub
          m_targetRpm = ShooterSubsystem.calculateRPMForDistanceToHUB(1.5);
        } else if (visionOverride == 2) {
          // Shoot from Tower
          m_targetRpm = ShooterSubsystem.calculateRPMForDistanceToHUB(1.75);
        }
        // We're overriding vision, so let driver drive.
        if (m_rotSupplier != null) {
          rotationSpeed = m_rotSupplier.getAsDouble();
        }
        translationX = m_translationXSupplier.getAsDouble();
        translationY = m_translationYSupplier.getAsDouble();
      } else {
        if (m_vision.hasTarget()) {
          double[] aimInfo = m_vision.getTargetAimInfo(m_drive.getRobotRelativeSpeeds(), m_aimPID);
          rotationSpeed = aimInfo[0];
          m_targetRpm = aimInfo[1];
          if (Math.abs(rotationSpeed) < 0.05) {
            m_isAimed = true;
          } else {
            m_isAimed = false;
          }
          m_noTarget = false;
        } else {
          if (!m_noTarget) {
            m_overrideStartTime = Timer.getFPGATimestamp();
            m_noTarget = true;
          } else {
            if (Timer.getFPGATimestamp() - m_overrideStartTime > 1) {
              // Automatic vision override.
              m_isAimed = true;
            }
          }
        }
        // Allow driving.
        translationX = m_translationXSupplier.getAsDouble() * 0.8;
        translationY = m_translationYSupplier.getAsDouble() * 0.8;
      }

      SmartDashboard.putNumber("Subsystems/Vision/Auto/RotationSpeed", rotationSpeed);
      SmartDashboard.putNumber("Subsystems/Vision/Auto/TargetRPM", m_targetRpm);
      SmartDashboard.putBoolean("Subsystems/Vision/Auto/IsAimed", m_isAimed);
      
      // Aim Robot
      if (translationX == 0 && translationY == 0 && rotationSpeed == 0) {
        m_drive.setX();
      } else {
        m_drive.drive(translationX, translationY, rotationSpeed, true);
      }

      // Rev the shooter
      m_shooter.runShooterRPM(m_targetRpm);

      if ((m_isAimed || visionOverride != 0) && /* m_isAdjusted && */ m_shooter.isAtSpeed()) {
        m_shooter.runFeeder(ShooterConstants.kFeederPower);
      } else {
        m_shooter.runFeeder(-0.07);
      }
  }

  @Override
  public void end(boolean interrupted) {
      // Stop the shooter when the button is released, or return to an idle RPM
      m_shooter.stop();
  }
}
