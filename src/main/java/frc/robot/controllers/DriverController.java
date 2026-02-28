package frc.robot.controllers;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OperatorConstants;

public class DriverController extends GenericHID {

  SlewRateLimiter limiterX = new SlewRateLimiter(1);
  SlewRateLimiter limiterY = new SlewRateLimiter(1);
  
  public DriverController(int port) {
    super(port);
  }

  // Axis
  //private final double k_triggerActivationThreshold = 0.5;
  //private double k_lastTriggerValue = 0.0;

  // Drive
  public double getLeftY(){
  return limiterX.calculate(getRawAxis(1));
  }
  public double getLeftX(){
  return limiterY.calculate(getRawAxis(0));
  }
  public double getRightX(){
  return getRawAxis(4);
  }
  public boolean getWantsHalfSpeedMode() {
    return getRawAxis(OperatorConstants.kHalfSpeedAxis) >= 0.5;
  }
  
  public boolean getWantsGyroReset(){
    return this.getRawButton(OperatorConstants.kResetGyroButton);
  }

  public boolean getWantsAimAndDrive(){
    return this.getRawButton(OperatorConstants.kAimAndDriveButton);
  }

  // Climber
  // TODO: Modify these once climber controls are set.
  public boolean getClimberUp(){
    return this.getRawButton(ClimberConstants.kClimberUpButton);
  }

  public boolean getClimberDown(){
    return this.getRawButton(ClimberConstants.kClimberDownButton);
  }

  public void outputTelemetry() {
    //SmartDashboard.putNumber(m_smartDashboardKey + "Forward", getForwardAxis());
    //SmartDashboard.putNumber(m_smartDashboardKey + "Turn", getTurnAxis());
  }
}
