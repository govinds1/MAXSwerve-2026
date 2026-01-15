package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;

public class DriverController extends GenericHID {
  
  public DriverController(int port) {
    super(port);
  }

  // Axis
  //private final double k_triggerActivationThreshold = 0.5;
  //private double k_lastTriggerValue = 0.0;

  // Drive
  public double getLeftY(){
  return getRawAxis(1);
  }
  public double getLeftX(){
  return getRawAxis(0);
  }
  public double getRightX(){
  return getRawAxis(4);
  }
  public boolean getWantsHalfSpeedMode() {
     return getRawAxis(2) > 0.5;
  }
  
  public boolean getWantsGyroReset(){
    return this.getRawButton(7);  //this should be the start button
  }

  // Climber
  // TODO: Modify these once climber controls are set.
  public boolean getClimberUp(){
    return this.getRawButton(4);  //this is the Y button
  }

  public boolean getClimberStopUp(){
    return this.getRawButtonReleased(4);
  }

  public boolean getClimberDown(){
    return this.getRawButton(1);  //this is the A button
  }
  
  public boolean getClimberStopDown(){
    return this.getRawButtonReleased(1);
  }

  public void outputTelemetry() {
    //SmartDashboard.putNumber(m_smartDashboardKey + "Forward", getForwardAxis());
    //SmartDashboard.putNumber(m_smartDashboardKey + "Turn", getTurnAxis());
  }
}
