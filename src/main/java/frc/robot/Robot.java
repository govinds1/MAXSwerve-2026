// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Robot Container.
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.
    m_robotContainer = new RobotContainer();
    m_robotContainer.init();

    LimelightHelpers.SetIMUMode("limelight", 0);

    // Warm up path following.
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Ensure 
    if (this.isEnabled()) {
      // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
      // commands, running already-scheduled commands, removing finished or interrupted commands,
      // and running subsystem periodic() methods.  This must be called from the robot's periodic
      // block in order for anything in the Command-based framework to work.
      CommandScheduler.getInstance().run();
    }
    m_robotContainer.periodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.getDriveSubsystem().stop();
    m_robotContainer.getIntakeSubsystem().stop();
    m_robotContainer.getShooterSubsystem().stop();
    m_robotContainer.getClimberSubsystem().stop();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // Get selected autonomous command.
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
if (m_robotContainer.getDriverController().getWantsGyroReset()) {
      m_robotContainer.getDriveSubsystem().zeroHeading();
    }

    // Intake control.
    if (m_robotContainer.getOperatorController().getWantsRunIntakeRoller()) {
      m_robotContainer.getIntakeSubsystem().runRollerRPM();
    } else if (m_robotContainer.getOperatorController().getWantsReverseIntakeRoller()) {
      m_robotContainer.getIntakeSubsystem().reverseRoller();
    } else {
      m_robotContainer.getIntakeSubsystem().stopRoller();
    }

    // Extender control.
    if (m_robotContainer.getOperatorController().getWantsExtenderOut()) {
      m_robotContainer.getIntakeSubsystem().extend();
    } else if (m_robotContainer.getOperatorController().getWantsExtenderIn()) {
      m_robotContainer.getIntakeSubsystem().retract();
    } else {
      m_robotContainer.getIntakeSubsystem().stopExtender();
    }

    // Climber control.
    if (m_robotContainer.getDriverController().getClimberUpManual()) {
      m_robotContainer.getClimberSubsystem().raiseHook(true);
    } else if (m_robotContainer.getDriverController().getClimberDownManual()) {
      m_robotContainer.getClimberSubsystem().lowerHook(true);
    } else {
      m_robotContainer.getClimberSubsystem().stop();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}