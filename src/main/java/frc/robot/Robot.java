// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Autos;
import frc.robot.controllers.DriverController;
import frc.robot.simulation.Field;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Controller
  private final DriverController m_driverController = new DriverController(0);
  //private final OperatorController m_operatorController = new OperatorController(1);

  // Robot Container.
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  // For Simulation
  private final Field m_field = Field.getInstance();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.
    m_robotContainer = new RobotContainer();

    CameraServer.startAutomaticCapture();   //This image is upside down.  Using the Thread with rotated Image to adjust

    SmartDashboard.putStringArray("Auto List", Autos.autoNames);

    // Set up the Field2d object for simulation
    SmartDashboard.putData("Field", m_field);
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

    // TODO: Do we need this?
    m_robotContainer.getDriveSubsystem().periodic();

    // Update Robot Simulation
    updateSim();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // TODO: Stop other subsystems?
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    String selectedAutoName = SmartDashboard.getString("Auto Selector", Autos.autoNames[0]);
    m_autonomousCommand = Autos.getSelectedAuto(selectedAutoName, m_robotContainer.getDriveSubsystem());

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

    // TODO:
    // If we climbed in auto, drop the climber.
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.getDriveSubsystem().drive(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.05),
        -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.05),
        -MathUtil.applyDeadband(m_driverController.getRightX(), 0.05),
        true);
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

  private void updateSim() {
    // Update the odometry in the sim.
    m_field.setRobotPose(m_robotContainer.getDriveSubsystem().getPose());
  }
}