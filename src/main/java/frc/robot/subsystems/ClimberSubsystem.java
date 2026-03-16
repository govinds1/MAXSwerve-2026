// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private SparkMax m_motor = new SparkMax(ClimberConstants.kClimberMotorCanId, MotorType.kBrushless);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_motor.configure(Configs.Climber.climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Subsystems/Climber/Encoder/Position", m_motor.getEncoder().getPosition());
  }

  public void raiseHook() {
    m_motor.set(ClimberConstants.kClimbSpeed);
  }

  public void lowerHook() {
    m_motor.set(-ClimberConstants.kClimbSpeed);
  }

  public boolean isRaised() {
    return m_motor.getEncoder().getPosition() - ClimberConstants.kClimberUpPosition > 0;
  }

  public boolean isLowered() {
    return m_motor.getEncoder().getPosition() - ClimberConstants.kClimberDownPosition < 0;
  }

  public void stop() {
    m_motor.stopMotor();
  }
}
