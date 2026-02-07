// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.events.EventTrigger;

//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.VisionTargeting;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  //private final IntakeSubsystem m_intake = new IntakeSubsystem();
  //private final VisionTargeting m_vision = new VisionTargeting();
  //private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  // The driver's controller
  private final DriverController m_driverController = new DriverController(OperatorConstants.kDriverControllerPort);
  // The operator's controller
  private final OperatorController m_operatorController = new OperatorController(OperatorConstants.kOperatorControllerPort);

  //private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    //configureButtonBindings();

    // Configure default commands
    /*
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.kDriveDeadband),
                true),
            m_robotDrive));
    */

    // Register Named Commands
    // TODO: Register shoot, intake, intake and move, and climb commands
    //NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());

    // Register Event Triggers
    //new EventTrigger("intake").whileTrue(Commands.parallel(Commands.print("running intake")/*, TODO: Add intake command */));
    
    // Register teleop Triggers
    //getOperatorController().runIntake.whileTrue(Commands.run(getIntakeSubsystem().runRoller()))
    //  .onFalse(Commands.run(getIntakeSubsystem().stopRoller()));
    // TODO: Modify shoot trigger to run feeder too. Automate this process, run feeder after ShooterConstants.kShooterSpinUpTime.
    //getOperatorController().runShooter.onTrue(Commands.runOnce(() -> getShooterSubsystem().runShooterOpenLoop(ShooterConstants.kShooterPower)));
    //getOperatorController().stopShooter.onTrue(Commands.runOnce(() -> getShooterSubsystem().stopShooter()));
    //getOperatorController().runFeeder.onTrue(Commands.runOnce(() -> getShooterSubsystem().runFeeder(ShooterConstants.kFeederPower)))
      //.onFalse(Commands.runOnce(() -> getShooterSubsystem().stopFeeder()));

    // Build an auto chooser. This will use Commands.none() as the default option.
    //autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    //SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  // GETTERS //

  DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }

  /*IntakeSubsystem getIntakeSubsystem() {
    return m_intake;
  }*/

  /*VisionTargeting getVisionSubsystem() {
    return m_vision;
  }*/

  /*ClimberSubsystem getClimberSubystem() {
    return m_climber;
  }*/

  ShooterSubsystem getShooterSubsystem() {
    return m_shooter;
  }

  DriverController getDriverController() {
    return m_driverController;
  }

  OperatorController getOperatorController() {
    return m_operatorController;
  }

  /**
   * Use this to initialize subsystems as needed. Should be called on Robot Init!
   * 
   */
  public void init() {
    // Initialize Limelight.
    //m_vision.init();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Grab selected auto from SmartDashboard drop down menu
    //String selectedAutoName = SmartDashboard.getString("Auto Selector", Autos.autoNames[0]);
    //return Autos.getSelectedAuto(selectedAutoName, m_robotDrive);

    // Grab selected auto from SmartDashboard chooser.
    //return autoChooser.getSelected();
    return Commands.idle();
  }
}