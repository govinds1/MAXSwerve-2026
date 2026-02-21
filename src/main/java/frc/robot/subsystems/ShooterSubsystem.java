// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.pathplanner.lib.events.TriggerEvent;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
  private final SparkFlex m_shooterMotor;
  private final SparkFlex m_shooterMotorFollower;
  private final SparkFlex m_feederMotor;

  private final RelativeEncoder m_encoder;

  private final SparkClosedLoopController m_closedLoopController;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.035, 0.0007, 0);
  private final double RPM_TOLERANCE = 500; // Spped margin of error
  private double m_desiredRPM = 0;

  // Distance (meters) - RPM mapping
  public static final InterpolatingDoubleTreeMap rpmMap = InterpolatingDoubleTreeMap.ofEntries(
    Map.entry(1.0, 2000.0),
    Map.entry(2.0, 3100.0),
    Map.entry(3.0, 4500.0),
    Map.entry(5.0, 5800.0)
  );

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public ShooterSubsystem() {
    m_shooterMotor = new SparkFlex(ShooterConstants.kRightShooterMotorCanId, MotorType.kBrushless);
    m_shooterMotorFollower = new SparkFlex(ShooterConstants.kLeftShooterMotorCanId, MotorType.kBrushless);
    m_feederMotor = new SparkFlex(ShooterConstants.kFeederMotorCanId, MotorType.kBrushless);

    m_encoder = m_shooterMotor.getEncoder();
    m_closedLoopController = m_shooterMotor.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_shooterMotor.configure(Configs.Shooter.shooterMotorConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    SparkBaseConfig followerConfig = Configs.Shooter.shooterMotorConfig;
    followerConfig.follow(m_shooterMotor, ShooterConstants.kInvertFollower); // Set to follow main motor, and invert.
    m_shooterMotorFollower.configure(followerConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    m_feederMotor.configure(Configs.Shooter.feederMotorConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Subsytems/Shooter/Flywheel/RPMSetpoint", m_desiredRPM);
    SmartDashboard.putNumber("Subsytems/Shooter/Flywheel/RPMCurrent", m_encoder.getVelocity());
  }

  /**
   * Run flywheel to speed.
   * 
   * @param desiredRPM RPM to set as motor velocity setpoint.
   */
  public void runShooterRPM(double desiredRPM) {
    m_desiredRPM = desiredRPM;
    if (desiredRPM <= 0) {
        //stopShooter();
    } else {
        double ffVoltage = feedforward.calculate(desiredRPM);
        
        m_closedLoopController.setSetpoint(
            desiredRPM, 
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0, 
            ffVoltage, 
            ArbFFUnits.kVoltage
        );
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
   * @param desiredRPM RPM to set as motor velocity setpoint.
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
   * Given a distance to Hub in meters, calculate desired RPM of the flywheel - assuming stationary robot.
   * 
   * @param distanceMeters Distance to Hub in meters.
   * @returns calculated RPM based on interpolated distance mapping
   */
  public static double calculateRPMForDistanceToHUB(double distanceMeters) {
    return rpmMap.get(distanceMeters);
  }

  /**
   * Check if flywheel is at speed.
   * 
   * @returns true if motor has reached the velocity setpoint.
   */
  public boolean isAtSpeed() {
    return Math.abs(m_encoder.getVelocity() - m_desiredRPM) < RPM_TOLERANCE;
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