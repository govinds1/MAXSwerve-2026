// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private SparkMax m_motor = new SparkMax(ClimberConstants.kClimberMotorCanId, MotorType.kBrushless);
  private DigitalInput m_limitSwitch = new DigitalInput(0);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_motor.configure(Configs.Climber.climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Subsystems/Climber/Encoder/Position", m_motor.getEncoder().getPosition());
    SmartDashboard.putBoolean("Subsystems/Climber/IsRaised", isRaised());
    SmartDashboard.putBoolean("Subsystems/Climber/IsLowered", isLowered());
    SmartDashboard.putNumber("Subsystems/Climber/Current", m_motor.getOutputCurrent());
    SmartDashboard.putBoolean("Subsystems/Climber/LimitSwitch", m_limitSwitch.get());

    if (isLowered()) {
      m_motor.getEncoder().setPosition(0);
    }
  }

  public void raiseHook() {
    if (isRaised()) {
      m_motor.stopMotor();
    } else {
      m_motor.set(ClimberConstants.kClimbSpeed);
    }
  }

  public void lowerHook() {
    if (isLowered()) {
      m_motor.stopMotor();
    } else {
      m_motor.set(-ClimberConstants.kClimbSpeed);
    }
  }

  public boolean isRaised() {
    return Math.abs(m_motor.getEncoder().getPosition() - ClimberConstants.kClimberUpPosition) < 2 || m_motor.getEncoder().getPosition() - ClimberConstants.kClimberUpPosition > 0;
  }

  public boolean isLowered() {
    return !m_limitSwitch.get(); //|| m_motor.getEncoder().getPosition() < 5;
  }

  public void stop() {
    m_motor.stopMotor();
  }
}
