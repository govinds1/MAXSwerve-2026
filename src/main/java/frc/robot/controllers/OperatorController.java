package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.IntakeConstants;

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

}
