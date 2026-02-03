// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Collections;
import java.util.Set;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Helpers;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.DriveAutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.AprilTagConstants.Tag;
import frc.robot.Constants.AprilTagConstants.TagLocation;
import frc.robot.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    public RawFiducial getTagInView() {
        // Get fiducial tag in view.
        var tagsInView = LimelightHelpers.getRawFiducials("");
        if (tagsInView.length == 0) {
            // TODO: Set some indicator in dashboard that we can't see the tag.
            return null;
        } else if (tagsInView.length >= 2) {
            // TODO: Set some indicator in dashboard that we see too many tags.
            // This means the pipeline is configured incorrectly!
            DriverStation.reportWarning("Pipeline configured incorrectly! Ensure IDs are filtered and grouped.", null);
            return null;
        }
        return tagsInView[0];
    }

    // THIS ASSUMES STATIONARY ROBOT!
    public ChassisSpeeds getSpeedsToTag(TagLocation location)
    {
        if (!canSee(location)) {
            // TODO: Set some indicator in dashboard that we can't see the tag.
            return null;
        }
        // TODO: Set the proper pipeline to target this location
        
        // Get fiducial tag in view.
        RawFiducial tag = getTagInView();

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
    
    // https://blog.eeshwark.com/robotblog/shooting-on-the-fly - uses a turret, we can use similar math to adjust our robot rotation.
    public ShootingInfo getHubAimInfo(Pose2d currentRobotPose, ChassisSpeeds currentRobotSpeeds)
    {
    // This function will assume the currentRobotPose is accurate after AprilTag localization and calculate the proper adjustments 
    // for current speeds to determine an angle and shooter RPM.

    // Extract speeds to 2d vector.
    Translation2d robotVelVec = new Translation2d(currentRobotSpeeds.vxMetersPerSecond, currentRobotSpeeds.vyMetersPerSecond);

    // Update pose for camera latency! Calculate the pose where we actually will shoot.
    double latency = LimelightConstants.kCameraLatencySeconds;
    Translation2d futurePos = currentRobotPose.getTranslation().plus(
        new Translation2d(robotVelVec.getX(), robotVelVec.getY()).times(latency)
    );

    // Determine alliance hub.
    Translation2d goalLocation = (Helpers.onRedAlliance()) ? FieldConstants.kRedHub : FieldConstants.kBlueHub;
    // TODO: Get visible Hub tag, if exists, and compare pose with goal location.
    // Get target vector, from robot pose to goal pose.
    Translation2d targetVec = goalLocation.minus(futurePos);
    double dist = targetVec.getNorm();

    // Calculate ideal shot, assuming stationary.
    // TODO: Ensure the calculate function returns horizontal speed! Or calculate it here.
    double idealHorizontalBallSpeed = ShooterSubsystem.calculateExitVelocityForDistanceToHub(dist); // meters/sec

    // Calculate shot vector accounting for current robot speeds and desired ball speed.
    Translation2d shotVec = targetVec.div(dist).times(idealHorizontalBallSpeed).minus(robotVelVec);

    // Convert to controls.
    double robotAngle = shotVec.getAngle().getRadians();
    double newHorizontalSpeed = shotVec.getNorm();

    ShootingInfo shot = new ShootingInfo();
    shot.pose = new Pose2d(futurePos.getX(), futurePos.getY(), new Rotation2d(robotAngle));
    shot.shootingRPM = ShooterSubsystem.calcRPMForExitHorizontalVelocity(newHorizontalSpeed);
    return shot;
    }

    public static class ShootingInfo
    {
        public Pose2d pose;
        public double shootingRPM;
    }
}