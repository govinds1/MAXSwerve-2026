// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Helpers;
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

  private final DoubleSupplier m_visionOverrideSupplier;

  private final PIDController m_aimPID = new PIDController(0.03, 0.0, 0); // tuned for Limelight Tx values. (0.05, 0.0, 0.005)
  private double m_targetRpm;
  private boolean m_isAimed; 
  private boolean m_isAdjusted;
  private double m_overrideStartTime = 0;
  private boolean m_noTarget = false;
  private double m_tagToHubOffset = 0;
  private Rotation2d m_aimedRotation;

  public AimClosedLoop(DriveSubsystem drive, ShooterSubsystem shooter, VisionTargeting vision, 
                          DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier visionOverrideSupplier) {
    m_drive = drive;
    m_shooter = shooter;
    m_vision = vision;
    m_translationXSupplier = xSupplier;
    m_translationYSupplier = ySupplier;
    m_visionOverrideSupplier = visionOverrideSupplier;
    m_isAimed = false;
    m_targetRpm = 18000;

    addRequirements(m_drive, m_shooter);

    SmartDashboard.putNumber("Subsystems/Vision/Auto/RotationSpeed", 0);
    SmartDashboard.putNumber("Subsystems/Vision/Auto/TargetRPM", 0);
    SmartDashboard.putBoolean("Subsystems/Vision/Auto/IsAimed", false);
  }

  @Override
  public void initialize() {
    m_isAimed = false;
    m_targetRpm = 18000;
    m_aimedRotation = null;
  }

  @Override
  public void execute() {
      double translationX = m_translationXSupplier.getAsDouble();
      double translationY = m_translationYSupplier.getAsDouble();
      
      double rotationSpeed = 0.0;

      double visionOverride = m_visionOverrideSupplier.getAsDouble();
      if (visionOverride != 0) {
        if (visionOverride == 1) {
          // Shoot from Hub
          m_targetRpm = ShooterSubsystem.calculateRPMForDistanceToHUB(0.8);
        } else {
          // Shoot from Tower
          m_targetRpm = ShooterSubsystem.calculateRPMForDistanceToHUB(2.2);
        }
      } else if (!m_isAimed) {
        if (m_vision.hasTarget()) {
          rotationSpeed = m_aimPID.calculate(m_vision.getTx(), 0.0);
          if (Math.abs(rotationSpeed) < 0.05) {
            m_isAimed = true;
            rotationSpeed = 0;
          } else {
            // Redundnant for now, since we do not allow shooting on the move.
            m_isAimed = false;
          }
          double distance = m_vision.getDistanceToTargetMeters();
          m_targetRpm = ShooterSubsystem.calculateRPMForDistanceToHUB(distance);
          m_noTarget = false;
        } else {
          if (!m_noTarget) {
            m_overrideStartTime = Timer.getFPGATimestamp();
            m_noTarget = true;
          } else {
            if (Timer.getFPGATimestamp() - m_overrideStartTime > 1) {
              m_isAimed = true;
              m_targetRpm = ShooterSubsystem.calculateRPMForDistanceToHUB(2.2);
            }
          }
        } 
      } else if (!m_isAdjusted) {
        // TODO: TEST THIS! This is extremely rough code, and assumes constant offset no matter where robot is.
        // Limelight's Tx value range is -27 to 27 degrees (for full window, -13.5 to 13.5 for our half window), so should be a small enough number.

        if (m_noTarget) {
          // Entered auto vision override.
          m_isAdjusted = true;
        } else {
          if (m_aimedRotation == null) {
            m_aimedRotation = m_drive.getHeadingRotation();
            double currentAngleDegrees = Helpers.modDegrees(m_aimedRotation.getDegrees());
            if (currentAngleDegrees < 45) {
              //m_tagToHubOffset = ((currentAngleDegrees - 0) / 45) * 1.5; 
              m_tagToHubOffset = MathUtil.inverseInterpolate(0, 45, currentAngleDegrees) * -3; // Range of 0 to -3 for angle of 0 to 45
            } else if (currentAngleDegrees > 315) {
              //m_tagToHubOffset = ((360 - currentAngleDegrees) / 45) * 1.5; 
              m_tagToHubOffset = MathUtil.inverseInterpolate(360, 315, currentAngleDegrees) * 3; // Range of 0 to 3 for angle of 360 to 315
            }
          }
          rotationSpeed = TurnToAngle.getRotSpeed(m_drive.getHeadingRotation().getRadians(), m_aimedRotation.plus(Rotation2d.fromDegrees(m_tagToHubOffset)).getRadians(), m_drive.m_thetaController);
          if (m_drive.m_thetaController.atSetpoint()) {
            m_isAdjusted = true;
            rotationSpeed = 0;
          } else {
            // Redundnant for now, since we do not allow shooting on the move.
            m_isAdjusted = false;
          }
        }
      }
      SmartDashboard.putNumber("Subsystems/Vision/Auto/RotationSpeed", rotationSpeed);
      SmartDashboard.putNumber("Subsystems/Vision/Auto/TargetRPM", m_targetRpm);
      SmartDashboard.putBoolean("Subsystems/Vision/Auto/IsAimed", m_isAimed);
      SmartDashboard.putBoolean("Subsystems/Vision/Auto/IsAdjusted", m_isAdjusted);
      
      // Aim Robot
      // TODO: Update with drive input when Vision aiming adjusts for robot speeds. See AimAndShootAuto/AimAtHubWhileDriving commands.
      if (translationX == 0 && translationY == 0 && rotationSpeed == 0) {
        m_drive.setX();
      } else {
        m_drive.drive(translationX, translationY, rotationSpeed, true);
      }

      // Rev the shooter
      m_shooter.runShooterRPM(m_targetRpm);
      //m_shooter.runShooterOpenLoop(0.6);

      if (m_isAimed && m_isAdjusted && m_shooter.isAtSpeed()) {
        m_shooter.runFeeder(ShooterConstants.kFeederPower);
      } else {
        m_shooter.stopFeeder();
      }
  }

  @Override
  public void end(boolean interrupted) {
      // Stop the shooter when the button is released, or return to an idle RPM
      m_shooter.stop();
  }
}
