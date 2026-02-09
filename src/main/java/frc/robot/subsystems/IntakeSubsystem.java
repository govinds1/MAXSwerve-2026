// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  SparkFlex m_rollerMotor = new SparkFlex(IntakeConstants.kIntakeRollerMotorCanId, MotorType.kBrushless);
  //SparkFlex m_extenderMotor = new SparkFlex(IntakeConstants.kIntakeExtenderMotorCanId, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_rollerMotor.configure(Configs.Intake.intakeMotorConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runRoller() {
    m_rollerMotor.set(IntakeConstants.kIntakeRollerSpeed);
  }

  public void stopRoller() {
    m_rollerMotor.stopMotor();
  }

  public void reverseRoller() {
    m_rollerMotor.set(-IntakeConstants.kIntakeRollerSpeed);
  }

  public void extend() {
    //m_extenderMotor.set(IntakeConstants.kIntakeExtenderMotorSpeed);
  }

  public void retract() {
    //m_extenderMotor.set(-IntakeConstants.kIntakeExtenderMotorSpeed);
  }

  public void stopExtender() {
    //m_extenderMotor.stopMotor();
  }

  public void stop() {
    stopRoller();
    stopExtender();
  }
}
