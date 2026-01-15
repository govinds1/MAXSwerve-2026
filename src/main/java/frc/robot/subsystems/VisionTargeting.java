// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Collections;
import java.util.Set;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.AprilTagConstants.Tag;
import frc.robot.Constants.AprilTagConstants.TagLocation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionTargeting extends SubsystemBase {

    private TagLocation m_seekingLocation;
    private Alliance m_currentAlliance;

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

        var currentAlliance = DriverStation.getAlliance();
        if (currentAlliance.isPresent())
        {
            m_currentAlliance = currentAlliance.get();
        }
    }

    @Override
    public void periodic() {
        // THIS FUNCTION WILL BE CALLED WHEN DISABLED AND ENABLED

        // Can we see the current desired location?
        if (canSee(m_seekingLocation))
        {
            // TODO: Put to smart dashboard - turn on some indicator for driver.
        }
    }

    public Set<Tag> getTagsForLocation(TagLocation location)
    {
        Set<Tag> tagsAtLocation = Collections.emptySet();
        for (Tag tag : AprilTagConstants.tags)
        {
            if (tag.m_location == location && tag.m_alliance == m_currentAlliance)
            {
                tagsAtLocation.add(tag);
            }
        }
        return tagsAtLocation;
    }

    public boolean canSee(TagLocation location)
    {
        // TODO: 
        // Get Tags to look for - use getTagsForLocation
        // Set limelight to filter for Tags.
        // Return true if Tags visible, otherwise false.
        return false;
    }

    public ShootingInfo aimAtHub()
    {
        // TODO:
        // Get 1 or 2 Hub Tags that are visible, if applicable. With visible tags, calculate pose of CENTER of Hub.
        // Given an ideal shooting RPM and ideal (minimal) movement time, calculate a pose to move the robot to in order to aim at hub.
        ShootingInfo shot = new ShootingInfo();
        shot.pose = Pose2d.kZero;
        shot.shootingRPM = 0;
        shot.shotReference = TagLocation.kHubClose;
        return shot;
    }

    public class ShootingInfo
    {
        Pose2d pose;
        double shootingRPM;
        TagLocation shotReference;
    }

}