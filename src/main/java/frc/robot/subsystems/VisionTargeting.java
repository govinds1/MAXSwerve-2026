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
import frc.robot.Constants.DriveAutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.AprilTagConstants.Tag;
import frc.robot.Constants.AprilTagConstants.TagLocation;
import frc.robot.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    public Set<Tag> getHubShootingTags()
    {
        Set<Tag> tags = getTagsForLocation(TagLocation.kHubClose);
        tags.addAll(getTagsForLocation(TagLocation.kHubLeft));
        tags.addAll(getTagsForLocation(TagLocation.kHubRight));
        return tags;
    }

    public boolean canSee(TagLocation location)
    {
        // TODO: 
        // Get Tags to look for - use getTagsForLocation
        // Set limelight to filter for Tags.
        // Return true if Tags visible, otherwise false.
        return false;
    }

    public ChassisSpeeds getSpeedsToTag(TagLocation location)
    {
        if (!canSee(location)) {
            // TODO: Set some indicator in dashboard that we can't see the tag.
            return new ChassisSpeeds();
        }
        // TODO: Set the proper pipeline to target this location
        
        // Get fiducial tag in view.
        var tagsInView = LimelightHelpers.getRawFiducials("");
        if (tagsInView.length == 0) {
            // TODO: Set some indicator in dashboard that we can't see the tag.
            return new ChassisSpeeds();
        } else if (tagsInView.length >= 2) {
            // TODO: Set some indicator in dashboard that we see too many tags.
            // This means the pipeline is configured incorrectly!
            DriverStation.reportWarning("Pipeline configured incorrectly! Ensure IDs are filtered and grouped.", null);
            return new ChassisSpeeds();
        }
        RawFiducial tag = tagsInView[0];

        // Get proportional controls to tag.
        double targetingForwardVelocity = tag.tync * DriveAutoConstants.kPXYController;
        double targetingAngularVelocity = tag.txnc * DriveAutoConstants.kPThetaController;
        // Convert to drivetrain speeds.
        targetingForwardVelocity *= DriveConstants.kMaxSpeedMetersPerSecond;
        targetingAngularVelocity *= DriveConstants.kMaxAngularSpeedRadiansPerSecond;
        // Invert since tx is positive when the target is to the right of the crosshair (meaning you would have to turn right, negative drive value).
        targetingAngularVelocity *= -1.0;
        // Invert since ty is positive when the target is above the crosshair (meaning you would have to drive backwards, negative drive value).
        targetingForwardVelocity *= -1.0;
        
        return new ChassisSpeeds(0, targetingForwardVelocity, targetingAngularVelocity);
    }

    public ShootingInfo getHubAimInfo(ChassisSpeeds currentRobotSpeeds)
    {
        // TODO:
        // Get 1 or 2 Hub Tags that are visible, if applicable. With visible tags, calculate pose of CENTER of Hub.
        // Given an ideal shooting RPM and ideal (minimal) movement time, calculate a pose to move the robot to in order to aim at hub.
        // TODO: Use drive odometry to figure out where Hub could be? Isolate to a single side of the Hub and find those Tags, also can turn the right direction if robot is not facing Hub.

        // Determine if we can see any of the Hub tags that we can shoot at.
        if (!canSee(TagLocation.kHubClose) && !canSee(TagLocation.kHubLeft) && !canSee(TagLocation.kHubRight))
        {
            // We cannot see the Hub, return null.
            return null;
        }

        // TODO:
        // Get tags that we can see and calculate the pose of the CENTER of the hub.

        // TODO:
        // Use currentRobotSpeeds in calculation.

        ShootingInfo shot = new ShootingInfo();
        shot.pose = Pose2d.kZero;
        shot.shootingRPM = 0;
        shot.shotReference = TagLocation.kHubClose;
        return shot;
    }

    public class ShootingInfo
    {
        public Pose2d pose;
        public double shootingRPM;
        public TagLocation shotReference;
    }

}