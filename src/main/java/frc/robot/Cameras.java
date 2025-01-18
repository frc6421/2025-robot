// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.annotation.Target;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/** Add your docs here. */
public class Cameras implements Sendable{

    public static PhotonCamera camera;
    private static double[] offsets; // XYZ, PITCH, YAW
    private static double yaw = 0;
    private static double pitch = 0;
    private static double[] poseSendable = {0,0,0,0};
    private static Pose2d pose2d = new Pose2d();
    public static double timeStamp = 0;
    
    public static final AprilTagFieldLayout tagLayout = null; // need to find way to get 2025 field tags

    public Cameras(PhotonCamera camera, double[] offsets) {
        this.camera = camera;
        this.offsets = offsets;

        SendableRegistry.add(this, "Cameras");
        Shuffleboard.getTab("Cameras").add(this);
    }

    private final static Transform3d robotToCam = new Transform3d( 
        new Translation3d(offsets[0], offsets[1], offsets[2]),
        new Rotation3d(0, offsets[3], offsets[4]));

    private final static PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
        tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

    public static boolean hasTarget() {
        // Gets the Pipeline, of which, we get the last element, then check if that element is valid 
        return camera.getAllUnreadResults().get(camera.getAllUnreadResults().size() - 1).hasTargets();
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'initSendable'");
    }
}
