// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Vision {
  private final DriveSubsystem drive;

  private final PhotonCamera camera1 = new PhotonCamera(VisionConstants.cam1);
  private final PhotonCamera camera2 = new PhotonCamera(VisionConstants.cam2);

  private PhotonPoseEstimator poseEstimator1 = new PhotonPoseEstimator(VisionConstants.kTagLayout,
      VisionConstants.kRobotToCamOne);
  private PhotonPoseEstimator poseEstimator2 = new PhotonPoseEstimator(VisionConstants.kTagLayout,
      VisionConstants.kRobotToCamTwo);

  private boolean isDriverMode;

  /** Creates a new Vision object. */
  public Vision(DriveSubsystem drive) {
    this.drive = drive;
    this.isDriverMode = false;
  }

  public void periodic() {
    // This method will be called once per scheduler run
    var result1 = camera1.getAllUnreadResults();
    var result2 = camera2.getAllUnreadResults();

    /*
     * Go through all tags red from PhotonVision per camera,
     * estimate posistion and add it to kalman filter in drive.
     */
    Optional<EstimatedRobotPose> estimation1;
    for (int i = 0; i < result1.size(); i++) {
      estimation1 = poseEstimator1.estimateCoprocMultiTagPose(result1.get(i));
      if (estimation1.isEmpty()) {
        estimation1 = poseEstimator1.estimateLowestAmbiguityPose(result1.get(i));
      }

      if (estimation1.isPresent()) {
        drive.addVisionMeasurement(estimation1.get(), calculateStdDevs(result1.get(i), 0));
      }
    }

    Optional<EstimatedRobotPose> estimation2;
    for (int i = 0; i < result2.size(); i++) {
      estimation2 = poseEstimator2.estimateCoprocMultiTagPose(result2.get(i));
      if (estimation2.isEmpty()) {
        estimation2 = poseEstimator2.estimateLowestAmbiguityPose(result2.get(i));
      }

      if (estimation2.isPresent()) {
        drive.addVisionMeasurement(estimation2.get(), calculateStdDevs(result2.get(i), 1));
      }
    }
  }

  // Largely taken from the AdvantageKit template, props to 6328
  /**
   * Method to calculate standard deviations.
   *
   * @param result      Photonvision results from camera.
   * @param cameraIndex The camera you are getting results from.
   */
  private Matrix<N3, N1> calculateStdDevs(PhotonPipelineResult result, int cameraIndex) {
    int tagCount = result.getTargets().size();

    double avgDistance = result.getTargets().stream()
        .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
        .average()
        .orElse(0.0);

    double stdDevFactor = Math.pow(avgDistance, 2.0) / tagCount;

    double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
    double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;

    // If multi-tag solve, trust it more
    if (tagCount > 1) {
      linearStdDev *= VisionConstants.linearStdDevMegatag2Factor;
      angularStdDev *= VisionConstants.angularStdDevMegatag2Factor;
    }

    // Per-camera trust factor
    if (cameraIndex < VisionConstants.cameraStdDevFactors.length) {
      linearStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
      angularStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
    }

    return new Matrix<>(Nat.N3(), Nat.N1(), new double[] {
        linearStdDev,
        linearStdDev,
        angularStdDev
    });
  }

  public void toggleDriverCam() {
    camera2.setDriverMode(!isDriverMode);
  }
}