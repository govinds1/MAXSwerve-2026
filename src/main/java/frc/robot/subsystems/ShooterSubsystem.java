// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
  private final SparkFlex m_motor;
  private final SparkFlex m_motorFollower;

  private final RelativeEncoder m_encoder;

  private final SparkClosedLoopController m_closedLoopController;

  //private double m_desiredRPM = 0;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public ShooterSubsystem() {
    m_motor = new SparkFlex(ShooterConstants.kLeftShooterMotorCanId, MotorType.kBrushless);
    m_motorFollower = new SparkFlex(ShooterConstants.kRightShooterMotorCanId, MotorType.kBrushless);

    m_encoder = m_motor.getEncoder();

    m_closedLoopController = m_motor.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_motor.configure(Configs.Shooter.motorConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    SparkBaseConfig followerConfig = Configs.Shooter.motorConfig;
    followerConfig.follow(m_motor, ShooterConstants.kInvertFollower); // Set to follow main motor, and invert.
    m_motorFollower.configure(followerConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  /**
   * Run flywheel to speed.
   * 
   * @param desiredRPM RPM to set as motor velocity setpoint.
   */
  public void run(double desiredRPM) {
    //m_desiredRPM = desiredRPM;
    m_closedLoopController.setSetpoint(desiredRPM, ControlType.kVelocity);
  }

  /**
   * Run flywheel on open loop control.
   * 
   * @param desiredRPM RPM to set as motor velocity setpoint.
   */
  public void runOpenLoop(double power) {
    //m_desiredRPM = desiredRPM;
    m_motor.set(power);
  }

  /**
   * Stop flywheel.
   */
  public void stop() {
    m_motor.stopMotor();
  }

  /**
   * Given a desired exit HORIZONTAL velocity of the ball (in meters/sec), calculate RPM to spin flywheel.
   * 
   * @param desiredExitVelMPS Desired exit horizontal velocity in meters/sec
   * @returns calculated RPM of flywheel
   */
  public static double calcRPMForExitHorizontalVelocity(double desiredExitHorizVelMPS) {
    // Calculate total exit velocity using horizontal component and launch angle.
    double totalExitVelocity = desiredExitHorizVelMPS / Math.cos(ShooterConstants.kLaunchAngleRadians);
    // Solve for RPM:
    // exitVel = omega * wheelRadius
    // omega = exitVel / wheelRadius
    // Since desired omega is RPM, but exitVel is mps^2 and wheelRadius is m, we need to add some conversion to get RPM
    // omega * (2pi rads / 1 rev) * (1 min / 60 seconds) = exitVel / wheelRadius
    // omega = (60/2pi) * exitVel / wheelRadius
    return (60 / (2 * Math.PI)) * totalExitVelocity / (ShooterConstants.kFlyWheelDiameterMeters / 2);
  }

  /**
   * Given a distance to Hub in meters, calculate desired exit velocity of the ball (in meters/sec).
   * 
   * @param distanceMeters Distance to Hub in meters.
   * @returns calculated exit velocity of ball
   */
  public static double calculateExitVelocityForDistanceToHub(double distanceMeters) {
      // Starting formulas:
    // horizDist = exitVel * cos(launchAngle) * t
    // vertDist = exitVel * sin(launchAngle) * t - (1/2)*g*t^2
    // Solve for t:
    // t = horizDist /  (exitVel * cos(launchAngle))
    // Substitute:
    // vertDist = horizDist * tan(launchAngle) - (g * horizDist^2) / (2 * exitVel^2 * cos(launchAngle)^2)
    // Solve for exitVel:
    // exitVel = sqrt((g * horizDist^2) / ((2 * cos(launchAngle)^2) * (horizDist * tan(launchAngle) - vertDist)))
    double g = 9.8; // meters per second^2
    double numerator = g * distanceMeters * distanceMeters;
    double denominator = 2.0 * Math.pow(Math.cos(ShooterConstants.kLaunchAngleRadians), 2) * (distanceMeters * Math.tan(ShooterConstants.kLaunchAngleRadians) - ShooterConstants.kShotVerticalDistance);
    return Math.sqrt(numerator / denominator); // meters per second
  }

  /**
   * Check if flywheel is at speed.
   * 
   * @returns true if motor has reached the velocity setpoint.
   */
  public boolean isAtSpeed() {
    return m_closedLoopController.isAtSetpoint();
  }

  /**
   * Read flywheel speed.
   * 
   * @returns actual flywheel motor RPM.
   */
  public double getActualSpeedRPM() {
    return m_encoder.getVelocity();
  }
}