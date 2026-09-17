// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
  private final TalonFX m_shooterMotor;
  private final TalonFX m_shooterMotorFollower;
  private final SparkFlex m_feederMotor;

  //private final RelativeEncoder m_encoder;

  //private final SparkClosedLoopController m_closedLoopController;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.003, 0.000268, 0); // (0.003, 0.000283, 0);
  //private double m_desiredRPM = 0;
  private double m_desiredVelocity = 0;
  private VelocityVoltage velocityRequest = new VelocityVoltage(0);

  // Distance (meters) - velocity mapping
  public static final InterpolatingDoubleTreeMap velocityMap = InterpolatingDoubleTreeMap.ofEntries(
    // Tuned for Krakens (TODO!)
    // Should be in rotations per second (max/min: -512 - 512)
    Map.entry(1.0, 192.0),
    Map.entry(1.6, 236.8),
    Map.entry(1.7, 257.28),
    Map.entry(1.85, 288.0),
    Map.entry(2.0, 313.6),
    Map.entry(3.0, 326.4),
    Map.entry(5.0, 396.8)

    /* //Tuned for NEOs
    Map.entry(1.0, 15000.0),
    Map.entry(1.6, 18500.0),
    Map.entry(1.7, 20100.0),
    Map.entry(1.85, 22500.0),
    Map.entry(2.0, 24500.0),
    Map.entry(3.0, 25500.0),
    Map.entry(5.0, 31000.0)*/
  );

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public ShooterSubsystem() {
    m_shooterMotor = new TalonFX(ShooterConstants.kRightShooterMotorCanId);
    m_shooterMotorFollower = new TalonFX(ShooterConstants.kLeftShooterMotorCanId);
    m_feederMotor = new SparkFlex(ShooterConstants.kFeederMotorCanId, MotorType.kBrushless);

    m_shooterMotorFollower.setControl(new Follower(ShooterConstants.kRightShooterMotorCanId, MotorAlignmentValue.Opposed));

    /*
    //m_encoder = m_shooterMotor.getEncoder();
    //m_closedLoopController = m_shooterMotor.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_shooterMotor.configure(Configs.Shooter.shooterMotorConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    SparkBaseConfig followerConfig = Configs.Shooter.shooterMotorConfig;
    followerConfig.follow(m_shooterMotor, true); // Set to follow main motor, and invert.
    m_shooterMotorFollower.configure(followerConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    */

    m_feederMotor.configure(Configs.Shooter.feederMotorConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Subsytems/Shooter/Flywheel/RPSSetpoint", m_desiredVelocity);
    SmartDashboard.putNumber("Subsytems/Shooter/Flywheel/RPSCurrent", m_shooterMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Subsytems/Shooter/Flywheel/AtSpeed", isAtSpeed());
  }

  /**
   * Run flywheel to speed.
   * 
   * @param desiredvelocity velocity to set as motor velocity setpoint.
   */
  public void runShooterVelocity(double desiredVelocity) {
    m_desiredVelocity = desiredVelocity;
    if (desiredVelocity <= 0) {
        stopShooter();
    } else {
        double ffVoltage = feedforward.calculate(desiredVelocity);
        m_shooterMotor.setControl(velocityRequest.withVelocity(m_desiredVelocity).withFeedForward(ffVoltage));
        
        /*m_closedLoopController.setSetpoint(
            desiredRPM, 
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0, 
            ffVoltage, 
            ArbFFUnits.kVoltage
        );*/
    } 
  }

  public void runShooterVelocity(Supplier<Double> distanceToHubSupplier) {
    runShooterVelocity(ShooterSubsystem.calculateVelocityForDistanceToHUB(distanceToHubSupplier.get()));
  }

  public Command ShootStraightCommand(Supplier<Double> distanceToHubSupplier) {
    return Commands.run(
      () -> runShooterAuto(distanceToHubSupplier),
      this
    ).finallyDo(() -> stop());
  }

  public void runShooterAuto(Supplier<Double> distanceToHubSupplier) {
    runShooterVelocity(distanceToHubSupplier);
    if (isAtSpeed()) {
      runFeeder(ShooterConstants.kFeederPower);
    } else {
      runFeeder(-0.1);
    }
  }

  /**
   * Run flywheel on open loop control.
   * 
   * @param power Open loop power (-1 to 1)
   */
  public void runShooterOpenLoop(double power) {
    m_shooterMotor.set(power);
  }

  /**
   * Stop flywheel.
   */
  public void stopShooter() {
    m_shooterMotor.stopMotor();
  }

  /**
   * Run feeder on open loop control.
   * 
   * @param desiredVelocity velocity to set as motor velocity setpoint.
   */
  public void runFeeder(double power) {
    //m_desiredRPM = desiredRPM;
    m_feederMotor.set(power);
  }

  /**
   * Stop feeder.
   */
  public void stopFeeder() {
    m_feederMotor.stopMotor();
  }

  public void stop() {
    stopShooter();
    stopFeeder();
  }

  /**
   * Given a distance to Hub in meters, calculate desired velocity of the flywheel - assuming stationary robot.
   * 
   * @param distanceMeters Distance to Hub in meters.
   * @returns calculated velocity based on interpolated distance mapping (motor rots per second)
   */
  public static double calculateVelocityForDistanceToHUB(double distanceMeters) {
    return velocityMap.get(distanceMeters);
  }

  /**
   * Check if flywheel is at speed.
   * 
   * @returns true if motor has reached the velocity setpoint.
   */
  public boolean isAtSpeed() {
    return Math.abs(m_shooterMotor.getVelocity().getValueAsDouble() - m_desiredVelocity) < ShooterConstants.kRPSTolerance; // || (m_shooterMotor.getVelocity().getValueAsDouble() > m_desiredVelocity);
  }

  /**
   * Read flywheel speed.
   * 
   * @returns actual flywheel motor RPS (motor rotations per second).
   */
  public double getActualSpeedRPS() {
    //return m_encoder.getVelocity();
    return m_shooterMotor.getVelocity().getValueAsDouble();
  }
}