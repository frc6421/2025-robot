// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.List;
import java.util.Optional;

import javax.xml.crypto.Data;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/** Add your docs here. */
public class WarriorCamera implements Sendable {

    private final PhotonCamera camera;
    private double[] cameraOffsets; // XYZ, PITCH, YAW
    private double yaw = 0;
    private double pitch = 0;
    private Pose2d cameraPose2d = new Pose2d();
    private List<PhotonPipelineResult> cameraResult;
    private PhotonPipelineResult latestCameraResult = new PhotonPipelineResult();
    private Transform3d robotToCam;
    private final PhotonPoseEstimator poseEstimator;
    Optional<EstimatedRobotPose> cameraEstimatedPose;
    private Matrix<N3, N1> standardDeviation;

    public final static class CameraConstants {
        public final static double[] CAM_1_OFFSET = { .26, -.19, 0, .2613, .4363 }; // X, Y, Z, ROLL, PITCH, YAW
        public final static double[] CAM_2_OFFSET = { 0, 0, 0, 0, 0 };
        public final static double[] CAM_3_OFFSET = { 0, 0, 0, 0, 0 };
        public final static double[] CAM_4_OFFSET = { 0, 0, 0, 0, 0 };
        public final static double[] CAM_5_OFFSET = { 0, 0, 0, 0, 0 };
        public final static double[] CAM_6_OFFSET = { 0.28, .26, 0.23, Units.degreesToRadians(0),
                Units.degreesToRadians(-23.24), Units.degreesToRadians(-25.8) };

        public final static Transform3d BACK_RIGHT_TRANSFORM3D = new Transform3d(new Translation3d(-0.28, -.26, 0.23),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-23.24),
                        Units.degreesToRadians(-25.8 + 180.0)));
        private final static AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025Reefscape);
        private final static double MAXIMUM_X_POSE = TAG_LAYOUT.getFieldLength();
        private final static double MAXIMUM_Y_POSE = TAG_LAYOUT.getFieldWidth();
        private final static double APRILTAG_LIMIT_METERS = 5;
        private final static double MAXIMUM_AMBIGUITY = 100;
        private final static Matrix<N3, N1> LOW_SD = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10)); // TODO Why
                                                                                                            // is this
                                                                                                            // 10?
        private final static Matrix<N3, N1> HIGH_SD = VecBuilder.fill(0.9, 0.9, Units.degreesToRadians(10)); // TODO Why
                                                                                                             // is this
                                                                                                             // 10?
        public static final Transform2d ODOMETRY_BLUE_OFFSET = new Transform2d(Inches.of(0.0).magnitude(),
                Inches.of(0.0).magnitude(), new Rotation2d());
        public static final Transform2d ODOMETRY_RED_OFFSET = new Transform2d(Inches.of(0.0).magnitude(),
                Inches.of(0.0).magnitude(), new Rotation2d());
    }

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable cameraStateTable = inst.getTable("CameraState");
    private final StructPublisher<Pose3d> cameraPose = cameraStateTable.getStructTopic("Pose", Pose3d.struct).publish();

    public WarriorCamera(String cameraName, Transform3d offsets) {
        camera = new PhotonCamera(cameraName);

        poseEstimator = new PhotonPoseEstimator(
                CameraConstants.TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, offsets);

        refreshData();

        SendableRegistry.add(this, camera.getName());
        SmartDashboard.putData(this);

    }

    public final void refreshData() {

        cameraResult = camera.getAllUnreadResults();
        if (!cameraResult.isEmpty()) {
            latestCameraResult = cameraResult.get(cameraResult.size() - 1);
            cameraEstimatedPose = poseEstimator.update(latestCameraResult);
            if (!cameraEstimatedPose.isEmpty()) {
                cameraPose2d = cameraEstimatedPose.get().estimatedPose.toPose2d();
                cameraPose.accept(cameraEstimatedPose.get().estimatedPose);
            }
        }
    }

    public final Object[] getPipeline() {
        return cameraResult.toArray();
    }

    public boolean hasTarget() {
        return latestCameraResult.hasTargets();
    }

    private Distance getCameraDistance(Translation2d targetTranslation) {
        if (cameraEstimatedPose.isPresent()) {
            return Meters.of(
                    cameraEstimatedPose.get().estimatedPose.toPose2d().getTranslation().getDistance(targetTranslation));
        } else {
            return Meters.of(cameraPose2d.getTranslation().getDistance(targetTranslation));
        }
    }

    public Pose2d getPose2d() {
        return cameraPose2d;
    }

    public Matrix<N3, N1> getStandardDeviation() {
        return standardDeviation;
    }

    public double getTimer() {
        return latestCameraResult.getTimestampSeconds();
    }

    public AprilTagFieldLayout getTagFieldLayout() {
        return CameraConstants.TAG_LAYOUT;
    }

    public int getBestTagId() {
        return latestCameraResult.getBestTarget().fiducialId;
    }

    public double getPitch() {
        return latestCameraResult.getBestTarget().getPitch();
    }

    public double getYaw() {
        return latestCameraResult.getBestTarget().getYaw();
    }

    public boolean filterOdometry() {
        // Refresh cam values before filtering
        refreshData();
        // Is the robot in Teleop Enabled?

        if (!camera.isConnected()) {
            return false;
        } else if (!(cameraEstimatedPose.isPresent())) {
            return false;
        }

        // Pose in Field??
        if (cameraPose2d.getX() > CameraConstants.MAXIMUM_X_POSE ||
                cameraPose2d.getY() > CameraConstants.MAXIMUM_Y_POSE ||
                cameraPose2d.getX() < 0 ||
                cameraPose2d.getY() < 0) {
            DataLogManager.log("Out of Field");
            return false;
        }

        if (isTagReliable() && cameraEstimatedPose.get().targetsUsed.size() >= 2) {
            standardDeviation = CameraConstants.LOW_SD;
        } else {
            standardDeviation = CameraConstants.HIGH_SD;
        }

        Optional<DriverStation.Alliance> allianceColor = DriverStation.getAlliance();
        if (allianceColor.isPresent()) {
            if (allianceColor.get().equals(Alliance.Red)) {
                cameraPose2d.plus(CameraConstants.ODOMETRY_RED_OFFSET);
            } else {
                cameraPose2d.plus(CameraConstants.ODOMETRY_BLUE_OFFSET);
            }
        }
        return true;
    }

    public boolean isTagReliable() {
        if (latestCameraResult.hasTargets()) {
            PhotonTrackedTarget bestTarget = latestCameraResult.getBestTarget();
            int targetID = bestTarget.getFiducialId();
            Translation2d cameraTranslation2d = cameraPose2d.getTranslation();
            Translation2d targetTranslation2d = CameraConstants.TAG_LAYOUT.getTagPose(targetID).get().getTranslation()
                    .toTranslation2d();

            if (cameraTranslation2d.getDistance(targetTranslation2d) < CameraConstants.APRILTAG_LIMIT_METERS
                    && bestTarget.getPoseAmbiguity() < CameraConstants.MAXIMUM_AMBIGUITY) {
                return true;
            } else {
                return false;
            }
        }
        return false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(camera.getName());
        builder.addBooleanProperty(camera.getName() + " has target?", () -> latestCameraResult.hasTargets(), null);
    }
}
