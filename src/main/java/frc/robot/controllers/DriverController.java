package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Helpers;

public class DriverController extends GenericHID {

  SlewRateLimiter limiterX = new SlewRateLimiter(2.0);
  SlewRateLimiter limiterY = new SlewRateLimiter(2.0);
  public Trigger turnAway;
  public Trigger turnTowards;
  
  public DriverController(int port) {
    super(port);
    turnAway = new Trigger(this::getWantsTurnAwayDS);
    turnTowards = new Trigger(this::getWantsTurnTowardsDS);
  }

  // Axis
  //private final double k_triggerActivationThreshold = 0.5;
  //private double k_lastTriggerValue = 0.0;

  // Drive
  public double getLeftY() {
  return smoothDriveInput(getRawAxis(1), limiterY);
  }
  public double getLeftX() {
  return smoothDriveInput(getRawAxis(0), limiterX);
  }
  public double getRightX() {
  return smoothDriveInput(getRawAxis(4), null);
  }
  public boolean getWantsHalfSpeedMode() {
    return getRawAxis(OperatorConstants.kHalfSpeedAxis) >= 0.5;
  }
  
  public boolean getWantsGyroReset() {
    return this.getRawButton(OperatorConstants.kResetGyroButton);
  }

  public boolean getWantsAimAndDrive() {
    return this.getRawButton(OperatorConstants.kAimAndDriveButton);
  }

  public boolean getWantsTurnAwayDS() {
    return this.getPOV() == 0;
  }

  public boolean getWantsTurnTowardsDS() {
    return this.getPOV() == 180;
  }

  private double smoothDriveInput(double value, SlewRateLimiter limiter) {
    if (limiter != null) {
      value = limiter.calculate(value);
    }
    value = MathUtil.applyDeadband(value, OperatorConstants.kDriveDeadband);
    return Helpers.signedSquare(value);
  }

  // Climber
  // TODO: Modify these once climber controls are set.
  public boolean getClimberUp() {
    return this.getRawButton(ClimberConstants.kClimberUpButton);
  }

  public boolean getClimberDown() {
    return this.getRawButton(ClimberConstants.kClimberDownButton);
  }

  public void outputTelemetry() {
    //SmartDashboard.putNumber(m_smartDashboardKey + "Forward", getForwardAxis());
    //SmartDashboard.putNumber(m_smartDashboardKey + "Turn", getTurnAxis());
  }
}
