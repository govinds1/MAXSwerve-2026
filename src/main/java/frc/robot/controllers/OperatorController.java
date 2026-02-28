package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class OperatorController extends GenericHID {

  public Trigger runIntake;
  public Trigger runExtake;
  public Trigger extendIntake;
  public Trigger retractIntake;
  public Trigger runShooter;
  public Trigger stopShooter;
  public Trigger runFeeder;

  public OperatorController(int port) {
    super(port);
    runIntake = new Trigger(this::getWantsRunIntakeRoller);
    runExtake = new Trigger(this::getWantsReverseIntakeRoller);
    extendIntake = new Trigger(this::getWantsExtenderOut);
    retractIntake = new Trigger(this::getWantsExtenderIn);
    runShooter = new Trigger(this::getWantsRunShooter);
    stopShooter = new Trigger(this::getWantsStopShooter);
    runFeeder = new Trigger(this::getWantsRunFeeder);
  }

  // Intake
  // TODO: Add Intake button getters.
  public boolean getWantsRunIntakeRoller() {
    return this.getRawButton(IntakeConstants.kRollerRunButton);
  }

  public boolean getWantsReverseIntakeRoller() {
    return this.getRawButton(IntakeConstants.kRollerReverseButton);
  }

  public boolean getWantsExtenderOut() {
    return this.getRawAxis(IntakeConstants.kExtenderOutAxis) > 0.5;
  }

  public boolean getWantsExtenderIn() {
    return this.getRawAxis(IntakeConstants.kExtenderInAxis) > 0.5;
  }


  // Shooter
  public boolean getWantsRunShooter() {
    return this.getRawButton(ShooterConstants.kRunShooterButton);
  }

  public boolean getWantsStopShooter() {
    return this.getRawButton(ShooterConstants.kStopShooterButton);
  }

  public boolean getWantsBallSender(){
    return this.getRawButton(ShooterConstants.kSendBallsButton);  //Added to test BM Feb 19
  }

  // Feeder
  public boolean getWantsRunFeeder() {
    return this.getRawButton(ShooterConstants.kRunFeederButton);
  }
}
