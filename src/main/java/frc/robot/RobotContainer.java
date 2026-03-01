// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;

//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.events.EventTrigger;

//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AimClosedLoop;
import frc.robot.commands.StrafeCenterToTag;
import frc.robot.commands.TurnToAngle;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionTargeting;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
//import frc.robot.subsystems.VisionTargeting;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final VisionTargeting m_vision = new VisionTargeting();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  // The driver's controller
  private final DriverController m_driverController = new DriverController(OperatorConstants.kDriverControllerPort);
  // The operator's controller
  private final OperatorController m_operatorController = new OperatorController(OperatorConstants.kOperatorControllerPort);

  //private final SendableChooser<Command> autoChooser;

  AimClosedLoop m_aimCommand = new AimClosedLoop(
    getDriveSubsystem(), 
    getShooterSubsystem(), 
    getVisionSubsystem(), 
    () -> -getDriverController().getLeftY(), 
    () -> -getDriverController().getLeftX(),
    () -> getOperatorController().getWantsVisionOverride()
  );

  TurnToAngle m_turnAwayCommand = new TurnToAngle(
    getDriveSubsystem(), 
    Rotation2d.fromDegrees(0), 
    () -> -getDriverController().getLeftY(), 
    () -> -getDriverController().getLeftX()
  );

  TurnToAngle m_turnTowardsCommand = new TurnToAngle(
    getDriveSubsystem(), 
    Rotation2d.fromDegrees(180), 
    () -> -getDriverController().getLeftY(), 
    () -> -getDriverController().getLeftX()
  );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.driveWithJoystick(m_driverController),
        m_robotDrive
      )
    );
    // Register Named Commands
    NamedCommands.registerCommand("Shoot", m_aimCommand);
    NamedCommands.registerCommand("ExtendIntake", m_intake.extendAuto());
    NamedCommands.registerCommand("RetractIntake", m_intake.retractAuto());
    NamedCommands.registerCommand("RunIntake", Commands.runOnce(() -> m_intake.runRoller(), m_intake));
    NamedCommands.registerCommand("StopIntake", Commands.runOnce(() -> m_intake.stopRoller(), m_intake));

    NamedCommands.registerCommand("ClimberUp", Commands.runOnce(() -> m_climber.raiseHook(), m_climber));
    NamedCommands.registerCommand("ClimberDown", Commands.runOnce(() -> m_climber.lowerHook(), m_climber));

    NamedCommands.registerCommand("AlignToTag", new StrafeCenterToTag(m_robotDrive, m_vision));

    // Register Event Triggers
    new EventTrigger("RunIntake").whileTrue(Commands.runOnce(() -> m_intake.runRoller(), m_intake));
    new EventTrigger("StopIntake").whileTrue(Commands.runOnce(() -> m_intake.stopRoller(), m_intake));

    // Build an auto chooser. This will use Commands.none() as the default option.
    //autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    //SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void configureButtonBindings() {
    // Shooter Triggers
    getOperatorController().runShooter.whileTrue(m_aimCommand);

    // Turn to Angle Triggers
    getDriverController().turnAway.onTrue(m_turnAwayCommand.withTimeout(1.5));
    getDriverController().turnTowards.onTrue(m_turnTowardsCommand.withTimeout(1.5));
  }

  // GETTERS //

  DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }

  IntakeSubsystem getIntakeSubsystem() {
    return m_intake;
  }

  VisionTargeting getVisionSubsystem() {
    return m_vision;
  }

  ClimberSubsystem getClimberSubsystem() {
    return m_climber;
  }

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