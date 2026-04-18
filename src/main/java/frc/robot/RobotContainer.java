// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimClosedLoopAdvanced;
import frc.robot.commands.AutonSwerveTimeControlCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.StrafeCenterToTag;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionTargeting;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

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

  AimClosedLoopAdvanced m_aimWhileMovingCommand = new AimClosedLoopAdvanced(
    getDriveSubsystem(), 
    getShooterSubsystem(), 
    getVisionSubsystem(), 
    () -> -getDriverController().getLeftY() * 0.5,
    () -> -getDriverController().getLeftX() * 0.5,
    () -> -getDriverController().getRightX() * 0.75,
    () -> getOperatorController().getWantsVisionOverride()
  );

  /*Command m_alignForClimbDistanceCommand = Commands.sequence(
    // NOTE: Assumes we are close to climb entrance point and facing the correct way (away from driver station)
    // Drive back until we're confident we're at the back wall, then drive forward the correct distance.
    Commands.parallel(
      new ClimberCommand(m_climber, true),
      Commands.sequence(
        new AutonSwerveTimeControlCommand(m_robotDrive, -0.25, 0, 0, 2.0, true),
        new AutonSwerveDistanceConstantControlCommand(m_robotDrive, new Translation2d(Units.inchesToMeters(25.06), 0), new Rotation2d(0), true)
      )
    ),
    new AutonSwerveTimeControlCommand(m_robotDrive, 0, -0.1, 0, 1.0, true),
    new ClimberCommand(m_climber, false)
  );*/

  Command m_alignForClimbTimeCommand = Commands.sequence(
    // NOTE: Assumes we are close to climb entrance point and facing the correct way (away from driver station)
    // Drive right to hit tower, back until we're confident we're at the back wall, then drive forward until hook hits the bar.
    new AutonSwerveTimeControlCommand(m_robotDrive, 0, -0.06, 0, 1.4, true),
    //new AutonSwerveTimeControlCommand(m_robotDrive, 0, 0.06, 0, 0.1, true),
    new AutonSwerveTimeControlCommand(m_robotDrive, -0.06, 0, 0, 3.8, true),
    new AutonSwerveTimeControlCommand(m_robotDrive, 0, -0.06, 0, 1, true),
    new ClimberCommand(m_climber, false).withTimeout(4.5)
  );

  /*Command m_alignForClimbVisionCommand = Commands.sequence(
    // NOTE: Assumes we are in front of tower. This will try to climb on the right (facing driver station)
    // Aligns with AprilTag first, then uses distance control to strafe to climb bar.
    Commands.parallel(
      new ClimberCommand(m_climber, true),
      Commands.sequence(
        new TurnToAngle(m_robotDrive, new Rotation2d(Math.PI), () -> 0, () -> 0),
        new StrafeCenterToTag(m_robotDrive, m_vision, 1.0) // Distance here may not actually be meters... check dashboard in teleop.
      )
    ),
    new AutonSwerveDistanceConstantControlCommand(m_robotDrive, new Translation2d(0, 33.5), new Rotation2d(Math.PI), true),
    new AutonSwerveTimeControlCommand(m_robotDrive, -0.1, 0, 0, 1.5, true),
    new ClimberCommand(m_climber, false)
  );*/

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
    NamedCommands.registerCommand("Shoot", Autos.AimAndShootCommand(getDriveSubsystem(), getShooterSubsystem(), getVisionSubsystem(), getIntakeSubsystem()));
    NamedCommands.registerCommand("ShootQuick", Autos.AimAndShootCommand(getDriveSubsystem(), getShooterSubsystem(), getVisionSubsystem(), getIntakeSubsystem(), 3));
    NamedCommands.registerCommand("ShootStraight", Autos.ShootNoAimCommand(m_shooter, m_vision, m_intake));
    NamedCommands.registerCommand("RunIntake", m_intake.extendAuto(true));
    NamedCommands.registerCommand("RetractIntake", m_intake.retractAuto());
    NamedCommands.registerCommand("StopIntake", Commands.runOnce(() -> m_intake.stopRoller(), m_intake));
    NamedCommands.registerCommand("Intake", m_intake.extendAuto(true).finallyDo(() -> m_intake.stopRoller()));

    NamedCommands.registerCommand("RaiseClimb", new ClimberCommand(m_climber, true).withTimeout(5));
    NamedCommands.registerCommand("LowerClimb", new ClimberCommand(m_climber, false).withTimeout(5));
    NamedCommands.registerCommand("DoClimb", m_alignForClimbTimeCommand);

    NamedCommands.registerCommand("AlignToTag", new StrafeCenterToTag(m_robotDrive, m_vision));

    // Register Event Triggers
    //new EventTrigger("Intake").onTrue(m_intake.extendAuto(true));
    //new EventTrigger("Intake").onFalse(Commands.runOnce(() -> m_intake.stopRoller(), m_intake));
    
    /*
    new EventTrigger("ShootStraight").whileTrue(
      Commands.sequence(
        Autos.ShootNoAimCommand(m_shooter, m_vision, m_intake)
      )
    );
    */

    // Add PathPlannerAuto commands to storage container.
    List<String> ppAutoNames = AutoBuilder.getAllAutoNames();
    if (ppAutoNames != null) {
      for (String ppAutoName : ppAutoNames) {
        Autos.PPAutos.put(ppAutoName, AutoBuilder.buildAuto(ppAutoName));
      }
      Autos.autoNames.addAll(ppAutoNames);
    }
    // Push to dashboard drop-down.
    SmartDashboard.putStringArray("Auto List", Autos.autoNames.toArray(String[]::new));

    // Build an auto chooser. This will use Commands.none() as the default option.
    //autoChooser = AutoBuilder.buildAutoChooser();

    //SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void configureButtonBindings() {
    // Shooter Triggers
    getOperatorController().runShooter.whileTrue(m_aimWhileMovingCommand);

    // Climber Triggers
    getDriverController().raiseHook.onTrue(new ClimberCommand(m_climber, true).withTimeout(5.0));
    getDriverController().lowerHook.onTrue(new ClimberCommand(m_climber, false).withTimeout(5.0));
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
  public void init() {}

  public void periodic() {
    //SmartDashboard.putBoolean("Subsystems/Vision/PDHSwitchableChannel", m_pdh.getSwitchableChannel());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Grab selected auto from SmartDashboard drop down menu
    String selectedAutoName = SmartDashboard.getString("Auto Selector", Autos.autoNames.get(0));
    return Autos.getSelectedAuto(selectedAutoName, m_robotDrive, m_shooter, m_vision, m_intake);

    // Grab selected auto from SmartDashboard chooser.
    //return autoChooser.getSelected();
    //return Commands.idle();
  }
}