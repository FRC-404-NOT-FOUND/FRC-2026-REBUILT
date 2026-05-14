// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Vision {
  private final DriveSubsystem drive;

  private final PhotonCamera camera1 = new PhotonCamera(VisionConstants.kCam1Name);
  private final PhotonCamera camera2 = new PhotonCamera(VisionConstants.kCam2Name);

  private final PhotonPoseEstimator poseEstimator1 = new PhotonPoseEstimator(
      VisionConstants.kTagLayout, VisionConstants.kRobotToCam1);
  private final PhotonPoseEstimator poseEstimator2 = new PhotonPoseEstimator(
      VisionConstants.kTagLayout, VisionConstants.kRobotToCam2);

  private final StructArrayPublisher<Pose3d> cam1SeenTags = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Vision/Cam1DetectedTags", Pose3d.struct)
      .publish();
  private final StructArrayPublisher<Pose3d> cam2SeenTags = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Vision/Cam2DetectedTags", Pose3d.struct)
      .publish();

  private boolean isDriverMode;

  /** Creates a new Vision object. */
  public Vision(DriveSubsystem drive) {
    this.drive = drive;
    this.isDriverMode = false;
  }

  public void periodic() {
    var result1 = camera1.getAllUnreadResults();
    var result2 = camera2.getAllUnreadResults();

    List<Pose3d> cam1Poses = new ArrayList<>();
    List<Pose3d> cam2Poses = new ArrayList<>();

    for (PhotonPipelineResult result : result1) {
      Optional<EstimatedRobotPose> estimation = poseEstimator1.estimateCoprocMultiTagPose(result);
      if (estimation.isEmpty()) {
        estimation = poseEstimator1.estimateLowestAmbiguityPose(result);
      }
      if (estimation.isPresent()) {
        drive.addVisionMeasurement(estimation.get(), calculateStdDevs(result, 0));
      }
      result.getTargets().forEach(target ->
          VisionConstants.kTagLayout.getTagPose(target.getFiducialId()).ifPresent(cam1Poses::add));
    }

    for (PhotonPipelineResult result : result2) {
      Optional<EstimatedRobotPose> estimation = poseEstimator2.estimateCoprocMultiTagPose(result);
      if (estimation.isEmpty()) {
        estimation = poseEstimator2.estimateLowestAmbiguityPose(result);
      }
      if (estimation.isPresent()) {
        drive.addVisionMeasurement(estimation.get(), calculateStdDevs(result, 1));
      }
      result.getTargets().forEach(target ->
          VisionConstants.kTagLayout.getTagPose(target.getFiducialId()).ifPresent(cam2Poses::add));
    }

    cam1SeenTags.set(cam1Poses.toArray(new Pose3d[0]));
    cam2SeenTags.set(cam2Poses.toArray(new Pose3d[0]));
  }

  // Largely adapted from the AdvantageKit template (6328)
  /**
   * Calculates standard deviations for a vision measurement.
   *
   * @param result      Pipeline result from the camera.
   * @param cameraIndex Index of the camera (for per-camera trust factors).
   */
  private Matrix<N3, N1> calculateStdDevs(PhotonPipelineResult result, int cameraIndex) {
    int tagCount = result.getTargets().size();

    if (tagCount == 0) {
      return new Matrix<>(Nat.N3(), Nat.N1(), new double[] {
          Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
      });
    }

    double avgDistance = result.getTargets().stream()
        .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
        .average()
        .orElse(0.0);

    double stdDevFactor = Math.pow(avgDistance, 2.0) / tagCount;

    double linearStdDev = VisionConstants.kLinearStdDevBaseline * stdDevFactor;
    double angularStdDev = VisionConstants.kAngularStdDevBaseline * stdDevFactor;

    if (tagCount > 1) {
      linearStdDev *= VisionConstants.kLinearStdDevMultitagFactor;
      angularStdDev *= VisionConstants.kAngularStdDevMultitagFactor;
    }

    if (cameraIndex < VisionConstants.kCameraStdDevFactors.length) {
      linearStdDev *= VisionConstants.kCameraStdDevFactors[cameraIndex];
      angularStdDev *= VisionConstants.kCameraStdDevFactors[cameraIndex];
    }

    return new Matrix<>(Nat.N3(), Nat.N1(), new double[] {
        linearStdDev,
        linearStdDev,
        angularStdDev
    });
  }

  /** Toggles the intake camera between driver mode and vision mode. */
  public void toggleDriverCam() {
    isDriverMode = !isDriverMode;
    camera2.setDriverMode(isDriverMode);
  }
}
