package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class OperatorController extends GenericHID {

  public OperatorController(int port) {
    super(port);
  }

  // Intake
  // TODO: Add Intake button getters.
  public boolean getWantsRunIntakeRoller() {
    return this.getRawButton(IntakeConstants.kRollerRunButton);
  }

  public boolean getWantsExtenderOut() {
    return this.getRawButton(IntakeConstants.kExtenderOutButton);
  }

  public boolean getWantsExtenderIn() {
    return this.getRawButton(IntakeConstants.kExtenderInButton);
  }


  // Shooter
  // TODO: Add Shooter button getters.
  public boolean getWantsRunShooter() {
    return this.getRawButton(ShooterConstants.kRunShooterButton);
  }

  public boolean getWantsStopShooter() {
    return this.getRawButton(ShooterConstants.kStopShooterButton);
  }
}
