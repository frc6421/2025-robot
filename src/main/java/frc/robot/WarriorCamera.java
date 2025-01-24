// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.lang.annotation.Target;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class WarriorCamera implements Sendable{

    private final PhotonCamera camera;
    private double[] cameraOffsets; // XYZ, PITCH, YAW
    private double yaw = 0;
    private double pitch = 0;
    private double[] cameraPoseSendable = {0,0,0,0};
    private Pose2d cameraPose2d = new Pose2d();
    private List<PhotonPipelineResult> cameraResult;
    private PhotonPipelineResult latestCameraResult = new PhotonPipelineResult();
    private List<PhotonTrackedTarget> cameraTarget;
    public double timeStamp = 0;
    private Transform3d robotToCam;
    private final PhotonPoseEstimator poseEstimator;

    public static class CameraConstants {
        public static double[] CAM_1_OFFSET = {0,0,0,0,0};
        public static double[] CAM_2_OFFSET = {0,0,0,0,0};
        public static double[] CAM_3_OFFSET = {0,0,0,0,0};
        public static double[] CAM_4_OFFSET = {0,0,0,0,0};
        public static double[] CAM_5_OFFSET = {0,0,0,0,0};
        public static double[] CAM_6_OFFSET = {0,0,0,0,0};
    }
    
    private final static AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public WarriorCamera(String cameraName, double[] offsets) {
        camera = new PhotonCamera(cameraName);
        cameraOffsets = new double[] {
            offsets[0],
            offsets[1],
            offsets[2],
            offsets[3],
            offsets[4]
        };
        refreshData();

        robotToCam = new Transform3d( 
            new Translation3d(cameraOffsets[0], cameraOffsets[1], cameraOffsets[2]),
            new Rotation3d(0, cameraOffsets[3], cameraOffsets[4]));

        poseEstimator = new PhotonPoseEstimator(
            TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

        SendableRegistry.add(this, camera.getName());
        SmartDashboard.putData(this);
        
    }

    public final void refreshData() {
        
        cameraResult = camera.getAllUnreadResults();
        if (cameraResult.isEmpty() == false) {
        latestCameraResult = cameraResult.get(cameraResult.size() - 1);
        cameraPose2d = poseEstimator.update(latestCameraResult).get().estimatedPose.toPose2d();
        cameraTarget.addAll(latestCameraResult.getTargets());

        cameraPoseSendable = new double[]{
            cameraPose2d.getX(),
            cameraPose2d.getY(),
            cameraPose2d.getRotation().getDegrees()
        };
    }
    }

    public final Object[] getPipeline() {
        return cameraResult.toArray();
    }

    public boolean hasTarget() {
        return latestCameraResult.hasTargets();
    }
    
    private final Distance getCameraDistance(Translation2d targetTranslation) {
        Optional<EstimatedRobotPose> cameraEstimatedPose = poseEstimator.update(latestCameraResult);

        if(cameraEstimatedPose.isPresent()) {
            return Meters.of(cameraEstimatedPose.get().estimatedPose.toPose2d().getTranslation().getDistance(targetTranslation));
        } else {
            return Meters.of(cameraPose2d.getTranslation().getDistance(targetTranslation));
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(camera.getName());
    }
}
