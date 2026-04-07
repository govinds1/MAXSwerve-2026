// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  SparkFlex m_rollerMotor = new SparkFlex(IntakeConstants.kIntakeRollerMotorCanId, MotorType.kBrushless);
  //SparkFlex m_extenderMotor = new SparkFlex(IntakeConstants.kIntakeExtenderMotorCanId, MotorType.kBrushless);
  SparkMax m_extenderMotor = new SparkMax(IntakeConstants.kIntakeExtenderMotorCanId, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_rollerMotor.configure(Configs.Intake.intakeMotorConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    m_extenderMotor.configure(Configs.Intake.extenderMotorConfig, ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Subsytems/Intake/Extender/EncoderValue", m_extenderMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Subsytems/Intake/Roller/Velocity", m_rollerMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Subsytems/Intake/Roller/Setpoint", m_rollerMotor.getClosedLoopController().getSetpoint());
  }

  public void runRoller() {
    m_rollerMotor.set(IntakeConstants.kIntakeRollerSpeed);
  }

  public void runRollerRPM() {
    m_rollerMotor.getClosedLoopController().setSetpoint(2700, ControlType.kVelocity);
  }

  public void stopRoller() {
    m_rollerMotor.stopMotor();
  }

  public void reverseRoller() {
    m_rollerMotor.set(-IntakeConstants.kIntakeRollerSpeed);
  }

  public void extend() {
    m_extenderMotor.set(IntakeConstants.kIntakeExtenderExtendSpeed);
  }

  public void retract() {
    m_extenderMotor.set(-IntakeConstants.kIntakeExtenderRetractSpeed);
  }

  public void stopExtender() {
    m_extenderMotor.stopMotor();
  }

  public Command extendAuto(boolean andRun) {
    Command runIntakeCommand = (andRun) ? Commands.runOnce(() -> runRollerRPM()) : Commands.none();
    return Commands.sequence(
      Commands.runOnce(() -> this.extend(), this),
      Commands.waitSeconds(IntakeConstants.kIntakeExtendTime),
      runIntakeCommand,
      Commands.runOnce(() -> this.stopExtender(), this)
    );
  }

  public Command extendAuto() {
    return Commands.sequence(
      Commands.runOnce(() -> this.extend(), this),
      Commands.waitSeconds(IntakeConstants.kIntakeExtendTime),
      Commands.runOnce(() -> this.stopExtender(), this)
    );
  }

  public Command retractAuto() {
    return Commands.sequence(
      Commands.runOnce(() -> this.retract(), this),
      Commands.waitSeconds(IntakeConstants.kIntakeRetractTime),
      Commands.runOnce(() -> this.stopExtender(), this)
    );
  }

  public Command agitateAuto() {
    return Commands.repeatingSequence(
      Commands.runOnce(() -> this.retract(), this),
      Commands.waitSeconds(IntakeConstants.kIntakeAgitateTime),
      Commands.runOnce(() -> this.extend(), this),
      Commands.waitSeconds(IntakeConstants.kIntakeAgitateTime)
    ).andThen(
      Commands.runOnce(() -> this.stopExtender(), this)
    );
  }

  public void stop() {
    stopRoller();
    stopExtender();
  }
}
