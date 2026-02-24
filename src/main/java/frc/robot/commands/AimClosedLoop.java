// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveAutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Helpers;
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

  private final PIDController m_aimPID = new PIDController(0.05, 0.0, 0.005);
  private double m_targetRpm;
  private boolean m_isAimed; 
  private double m_overrideStartTime = 0;
  private boolean m_noTarget = false;

  public AimClosedLoop(DriveSubsystem drive, ShooterSubsystem shooter, VisionTargeting vision, 
                          DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    m_drive = drive;
    m_shooter = shooter;
    m_vision = vision;
    m_translationXSupplier = xSupplier;
    m_translationYSupplier = ySupplier;
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
  }

  @Override
  public void execute() {
      double translationX = m_translationXSupplier.getAsDouble();
      double translationY = m_translationYSupplier.getAsDouble();
      // Apply deadband.
      translationX = MathUtil.applyDeadband(translationX, OperatorConstants.kDriveDeadband);
      translationY = MathUtil.applyDeadband(translationY, OperatorConstants.kDriveDeadband);
      // Square inputs.
      translationX = Helpers.signedSquare(translationX);
      translationY = Helpers.signedSquare(translationY);
      
      double rotationSpeed = 0.0;

      if (m_vision.hasTarget() && !m_isAimed) {
          
          rotationSpeed = m_aimPID.calculate(m_vision.getTx(), 0.0);
          if (Math.abs(rotationSpeed) < 0.05) {
            m_isAimed = true;
            rotationSpeed = 0;
          } else {
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
            m_targetRpm = 18000;
          }
        }
        //m_isAimed = true;
      }
      SmartDashboard.putNumber("Subsystems/Vision/Auto/RotationSpeed", rotationSpeed);
      SmartDashboard.putNumber("Subsystems/Vision/Auto/TargetRPM", m_targetRpm);
      SmartDashboard.putBoolean("Subsystems/Vision/Auto/IsAimed", m_isAimed);
      
      // Aim Robot
      m_drive.drive(0, 0, rotationSpeed, true); 

      // Rev the shooter
      m_shooter.runShooterRPM(m_targetRpm);
      //m_shooter.runShooterOpenLoop(0.6);

      if (m_isAimed && m_shooter.isAtSpeed()) {
        m_shooter.runFeeder(ShooterConstants.kFeederPower);
      }
  }

  @Override
  public void end(boolean interrupted) {
      // Stop the shooter when the button is released, or return to an idle RPM
      m_shooter.stop();
  }
}
