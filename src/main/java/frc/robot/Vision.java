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

public class Vision {
  // Cameras
  private final PhotonCamera camera1 = new PhotonCamera(VisionConstants.cam1);
  private final PhotonCamera camera2 = new PhotonCamera(VisionConstants.cam2);

  // Pose estimator
  PhotonPoseEstimator poseEstimator1 = new PhotonPoseEstimator(VisionConstants.kTagLayout, VisionConstants.kRobotToCamOne);
  PhotonPoseEstimator poseEstimator2 = new PhotonPoseEstimator(VisionConstants.kTagLayout, VisionConstants.kRobotToCamTwo);

  // Add drive
  private final DriveSubsystem drive;

  /** Creates a new VisionSubsystem. */
  public Vision(DriveSubsystem drive) {
    this.drive = drive;
  }

  public void periodic() {
    // This method will be called once per scheduler run
    var result1 = camera1.getAllUnreadResults();
    var result2 = camera2.getAllUnreadResults();

    Optional<EstimatedRobotPose> estimation1;
    for (int i = 0; i < result1.size(); i++) {
      estimation1 = poseEstimator1.estimateCoprocMultiTagPose(result1.get(i));
      if (estimation1.isEmpty()) {
        estimation1 = poseEstimator2.estimateLowestAmbiguityPose(result1.get(i));
      }

      if (estimation1.isPresent()) {
        drive.addVisionMeasurement(estimation1);
      }
    }
    
    Optional<EstimatedRobotPose> estimation2;
    for (int i = 0; i < result2.size(); i++) {
      estimation2 = poseEstimator2.estimateCoprocMultiTagPose(result2.get(i));
      if (estimation2.isEmpty()) {
        estimation2 = poseEstimator2.estimateLowestAmbiguityPose(result2.get(i));
      }

      if (estimation2.isPresent()) {
        drive.addVisionMeasurement(estimation2);
      }
    }
  }
}
