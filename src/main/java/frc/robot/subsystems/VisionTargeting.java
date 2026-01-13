// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class VisionTargeting extends SubsystemBase {

    public VisionTargeting() {
        // Set a custom crop window for improved performance (-1 to 1 for each value)
        LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);


        // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
        LimelightHelpers.setCameraPose_RobotSpace("", 
            LimelightConstants.kForwardOffsetMeters,
            LimelightConstants.kSideOffsetMeters,
            LimelightConstants.kHeightOffsetMeters,
            LimelightConstants.kRollDegrees,
            LimelightConstants.kPitchDegrees,
            LimelightConstants.kYawDegrees
        );

        // Configure AprilTag detection
        //LimelightHelpers.SetFiducialIDFiltersOverride("", new int[]{1, 2, 3, 4}); // Only track these tag IDs
        LimelightHelpers.SetFiducialDownscalingOverride("", 1.5f);
    }

    // TODO: Implement functions.
    public void init() {
        // Turn off Limelight LED.
        LimelightHelpers.setLEDMode_ForceOff("");
    }
    public void periodic() {
        
    }

}