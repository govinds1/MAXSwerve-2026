// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autos;
//import frc.robot.commands.TeleopTargetAndShoot;
//import frc.robot.simulation.Field;

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

  // For Simulation
  //private final Field m_field = Field.getInstance();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.
    m_robotContainer = new RobotContainer();

    //CameraServer.startAutomaticCapture();   //This image is upside down.  Using the Thread with rotated Image to adjust

    SmartDashboard.putStringArray("Auto List", Autos.autoNames);

    // Set up the Field2d object for simulation
    //SmartDashboard.putData("Field", m_field);

    m_robotContainer.init();
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

    // Update Robot Simulation
    //updateSim();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // TODO: Stop other subsystems?
    m_robotContainer.getDriveSubsystem().stop();
    //m_robotContainer.getIntakeSubsystem().stop();
    m_robotContainer.getShooterSubsystem().stop();

    // When disabled, seed the Limelight's internal IMU with the external gyro input. 
    // Setting IMU mode of 1 enables seeding.
    // TODO:
    //LimelightHelpers.SetIMUMode("limelight", 1); // Seed internal IMU
  }

  @Override
  public void disabledPeriodic() {
    // TODO:
    // Seed Limelight with external gyro's heading.
    //LimelightHelpers.SetRobotOrientation("limelight", m_robotContainer.getDriveSubsystem().getHeadingDegrees(), 0, 0, 0, 0, 0);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    String selectedAutoName = SmartDashboard.getString("Auto Selector", Autos.autoNames[0]);
    m_autonomousCommand = Autos.getSelectedAuto(selectedAutoName, m_robotContainer.getDriveSubsystem());

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    // TODO:
    //LimelightHelpers.SetIMUMode("limelight", 4); // Use internal IMU + external IMU
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

    // TODO:
    //LimelightHelpers.SetIMUMode("limelight", 4); // Use internal IMU + external IMU
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // TODO: Add toggle/hold button for robot relative driving?
    /*
    TeleopTargetAndShoot command = new TeleopTargetAndShoot(
      m_robotContainer.getDriveSubsystem(),
      null, // m_robotContainer.getVisionSubsystem(), // TODO: Update when enabling vision.
      null, // m_robotContainer.getShooterSubsystem(), // TODO: Update when enabling shooter.
      m_robotContainer.getDriverController()
    );
    */
    // TODO: Get driver button to see if we want to target hub.
    boolean driveAndTarget = false;
    if (driveAndTarget) {
      // TODO: Run DriveTargetAndShoot command
      //CommandScheduler.getInstance().schedule(command);
    } else {
      /*
      if (command.isScheduled()) {
        CommandScheduler.getInstance().cancel(command);
      }
      */
      m_robotContainer.getDriveSubsystem().driveWithJoystick(m_robotContainer.getDriverController(), null);
    }

    // TODO: Replace with Triggers in RobotContainer constructor.

    // Intake control.
    /*if (m_robotContainer.getOperatorController().getWantsRunIntakeRoller()) {
      m_robotContainer.getIntakeSubsystem().runRoller();
    } else {
      m_robotContainer.getIntakeSubsystem().stop();
    }*/

    // Shooter control.
    if (m_robotContainer.getOperatorController().getWantsRunShooter()) {
      m_robotContainer.getShooterSubsystem().runShooterOpenLoop(ShooterConstants.kShooterPower);;
    } else if (m_robotContainer.getOperatorController().getWantsStopShooter()) {
      m_robotContainer.getShooterSubsystem().stopShooter();
    }
    // Feeder control.
    if (m_robotContainer.getOperatorController().getWantsRunFeeder()) {
      m_robotContainer.getShooterSubsystem().runFeeder(ShooterConstants.kFeederPower);
    } else {
      m_robotContainer.getShooterSubsystem().stopFeeder();
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

  /* TODO:\
  private void updateSim() {
    // Update the odometry in the sim.
    m_field.setRobotPose(m_robotContainer.getDriveSubsystem().getPose());
  }
  */
}