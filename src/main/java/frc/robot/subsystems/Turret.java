// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
    private final SparkFlex m_turretSpark;
    private final AbsoluteEncoder m_encoder;
    private final SparkClosedLoopController m_closedLoopController;

    private DriveSubsystem m_drive;

    private boolean m_aimAtGoal = false;
    private Rotation2d m_currentSetpoint;

    public Turret(DriveSubsystem drive) {
        m_turretSpark = new SparkFlex(TurretConstants.kTurretMotorCanId, MotorType.kBrushless);
        m_encoder = m_turretSpark.getAbsoluteEncoder();
        m_closedLoopController = m_turretSpark.getClosedLoopController();

        m_drive = drive;

        // Apply the respective configurations to the SPARK. Reset parameters before
        // applying the configuration to bring the SPARK to a known good state. Persist
        // the settings to the SPARK to avoid losing them on a power cycle.
        m_turretSpark.configure(Configs.Turret.config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        // Initialize WPILib PID controller. TODO: Decide whether we're using this or Spark's position control.
        var thetaController =
            new ProfiledPIDController(
                TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, TurretConstants.kControllerConstraints
            );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    // TODO: Implement functions.
    public void init() {
        // Initialize encoder, if relative?
        // Set position control to current/initialized angle.
        // Set m_currentSetpoint.
    }
    public void periodic() {
        // If m_aimAtGoal is true, find goal and aim at it.
        // Otherwise, maintain current setpoint (i.e. do nothing).
    }
    public void goToAngle(Rotation2d setpointAngle) {
        // Set position control to setpoint.
        m_currentSetpoint = setpointAngle;
    }
    public boolean atAngle(Rotation2d angle) {
        // Compare current position (from encoder) to angle.
        return true;
    }
    public boolean findHub() {
        // Returns true if hub is in view, otherwise return false.
        return false;
    }
    public Rotation2d getAngleToHub() {
        // Return angle offset from current angle to hub.
        return Rotation2d.kZero;
    }
}