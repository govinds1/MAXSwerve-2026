// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveAutoConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class VisionTargeting extends SubsystemBase {

    //private TagLocation m_seekingLocation;
    private final String limelightName = "limelight";

    public VisionTargeting() {
        // Set a custom crop window for improved performance (-1 to 1 for each value)
        LimelightHelpers.setCropWindow(limelightName, -0.75, 0.75, -0.1, 0.9);


        // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
        LimelightHelpers.setCameraPose_RobotSpace(limelightName, 
            LimelightConstants.kForwardOffsetMeters,
            LimelightConstants.kRightOffsetMeters,
            LimelightConstants.kHeightOffsetMeters,
            LimelightConstants.kRollDegrees,
            LimelightConstants.kPitchDegrees,
            LimelightConstants.kYawDegrees
        );

        // Configure AprilTag detection
        LimelightHelpers.SetFiducialDownscalingOverride(limelightName, 1.5f);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Subsystems/Vision/Has Target", hasTarget());
        SmartDashboard.putNumber("Subsystems/Vision/TX (Aiming)", getTx());
        SmartDashboard.putNumber("Subsystems/Vision/TY", getTy());
        SmartDashboard.putNumber("Subsystems/Vision/Calculated Distance (m)", getDistanceToTargetMeters());
        SmartDashboard.putNumber("Subsystems/Vision/Heartbeat", LimelightHelpers.getLimelightNTDouble(limelightName, "hb"));
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    public double getTx() {
        LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);
        
        if (fiducials.length >= 2) {
            double tx1 = fiducials[0].txnc;
            double tx2 = fiducials[1].txnc;
            
            return (tx1 + tx2) / 2.0; 
        } 
        else if (fiducials.length == 1) {
            return fiducials[0].txnc;
        }
        
        return 0.0; // No targets
    }

    public double getTy() {
        LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);
        
        if (fiducials.length >= 2) {
            double ty1 = fiducials[0].tync;
            double ty2 = fiducials[1].tync;
            
            return (ty1 + ty2) / 2.0; 
        } 
        else if (fiducials.length == 1) {
            return fiducials[0].tync;
        }
        
        return 0.0;
    }

    public double getDistanceToTargetMeters() {
        if (!hasTarget()) {
            return 0.0; 
        }
        
        // a2: The vertical angle to the target from the Limelight
        double targetOffsetAngleDegrees = getTy();
        
        // a1 + a2: Total angle from the ground
        double angleToGoalDegrees = LimelightConstants.kPitchDegrees + targetOffsetAngleDegrees;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        return (ShooterConstants.kHubHeightMeters - LimelightConstants.kHeightOffsetMeters) / Math.tan(angleToGoalRadians);
    }
    
    public double[] getTargetAimInfo(ChassisSpeeds currentRobotSpeeds, PIDController aimPID) {
        double[] aimInfo = new double[2];
        aimInfo[0] = 0;
        aimInfo[1] = ShooterSubsystem.calculateRPMForDistanceToHUB(1.9);
        // Return if we have no target
        if (!hasTarget()) {
            return aimInfo;
        }

        // Moving forward/backward, reduce/increase RPM respectively.
        double rpmOffset = currentRobotSpeeds.vxMetersPerSecond * DriveAutoConstants.kVelocityXToRPMOffset;
        // Moving left/right, aim right/left respectively.
        double aimOffset = currentRobotSpeeds.vyMetersPerSecond * DriveAutoConstants.kVelocityYToAimTxOffset;

        // Compute rotation command from PID controller
        double rotationSpeed = aimPID.calculate(getTx() + aimOffset, 0.0);

        // Compute RPM from distance
        double distanceToGoal = getDistanceToTargetMeters();
        double targetRpm = ShooterSubsystem.calculateRPMForDistanceToHUB(distanceToGoal) + rpmOffset;

        // Return aimInfo.
        aimInfo[0] = rotationSpeed;
        aimInfo[1] = targetRpm;
        return aimInfo;
    }
}