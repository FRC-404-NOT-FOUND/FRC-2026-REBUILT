// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.Navx;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;

import org.photonvision.EstimatedRobotPose;

public class DriveSubsystem extends SubsystemBase {
  private final MAXSwerveModule frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private final Navx gyro = new Navx(DriveConstants.kGyroPort, 100);
  private double gyroAngle;

  private final StructPublisher<Pose2d> robotPose = NetworkTableInstance.getDefault()
      .getStructTopic("Actual pose", Pose2d.struct).publish();

  private final StructArrayPublisher<Pose2d> rejectedPoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Vision/RejectedPoses", Pose2d.struct)
      .publish();

  private final List<Pose2d> rejectedPoseBuffer = new ArrayList<>();
  private int periodicDelay = 0;

  private final PIDController feedbackController = new PIDController(DriveConstants.kHeadingP, 0, 0);
  private double omega;

  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(gyro.getYaw().in(Degrees)),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
      }, new Pose2d());

  /**
   * Adds a vision pose estimate to the Kalman filter.
   *
   * @param visionPose Estimated pose from AprilTag vision.
   * @param stdDevs    Standard deviations representing how much to trust this measurement.
   */
  public void addVisionMeasurement(EstimatedRobotPose visionPose, Matrix<N3, N1> stdDevs) {
    Pose2d pose = visionPose.estimatedPose.toPose2d();

    if (pose.getX() < 0 || pose.getX() > VisionConstants.kTagLayout.getFieldLength()
        || pose.getY() < 0 || pose.getY() > VisionConstants.kTagLayout.getFieldWidth()) {
      rejectedPoseBuffer.add(pose);
      return;
    }

    if (Math.abs(visionPose.estimatedPose.getZ()) > VisionConstants.kMaxZError) {
      rejectedPoseBuffer.add(pose);
      return;
    }

    if (visionPose.targetsUsed.size() == 1
        && visionPose.targetsUsed.get(0).getPoseAmbiguity() > VisionConstants.kMaxAmbiguity) {
      rejectedPoseBuffer.add(pose);
      return;
    }

    poseEstimator.addVisionMeasurement(pose, visionPose.timestampSeconds, stdDevs);
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    gyro.enableOptionalMessages(true, true, true, false, false, false, false, false, false, false);

    feedbackController.enableContinuousInput(-Math.PI, Math.PI);
    feedbackController.setTolerance(Math.toRadians(DriveConstants.kHeadingToleranceDegrees));

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    RobotConfig config = PathPlannerConstants.config;
    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> pathplannerRelativeDrive(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(DriveConstants.kPathplannerTranslationP, 0.0, 0),
            new PIDConstants(DriveConstants.kPathplannerRotationP, 0.0, 0)),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        },
        this);
  }

  @Override
  public void periodic() {
    gyroAngle = gyro.getYaw().in(Degrees) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    poseEstimator.update(
        Rotation2d.fromDegrees(gyroAngle),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });
    robotPose.set(getPose());
    SmartDashboard.putNumber("Gyro raw: ", gyroAngle);
    rejectedPoses.set(rejectedPoseBuffer.toArray(new Pose2d[0]));
    periodicDelay++;
    if (periodicDelay % 25 == 0) {
      rejectedPoseBuffer.clear();
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the pose estimator to the specified pose.
   *
   * @param pose The pose to set.
   */
  public void resetPose(Pose2d pose) {
    double currentAngle = gyro.getYaw().in(Degrees) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(currentAngle),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        },
        pose);
  }

  /** Returns robot-relative ChassisSpeeds. */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        frontLeft.getState(),
        frontRight.getState(),
        rearLeft.getState(),
        rearRight.getState());
  }

  /**
   * Robot-relative drive for use with PathPlanner.
   *
   * @param speeds ChassisSpeeds to apply to the modules.
   */
  public void pathplannerRelativeDrive(ChassisSpeeds speeds) {
    var states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(states);
  }

  /**
   * Drives the robot using joystick inputs.
   *
   * @param xSpeed        Forward speed (-1 to 1).
   * @param ySpeed        Sideways speed (-1 to 1).
   * @param rot           Rotation rate (-1 to 1).
   * @param fieldRelative Whether speeds are field-relative.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var alliance = DriverStation.getAlliance();
    int alwaysBlueInvert = (alliance.isPresent() && alliance.get() == Alliance.Red) ? -1 : 1;

    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = (omega != 0) ? omega : (rot * DriveConstants.kMaxAngularSpeed);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered * alwaysBlueInvert,
                ySpeedDelivered * alwaysBlueInvert,
                rotDelivered,
                getHeading())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Sets wheels into an X formation to prevent movement. */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /** Points the robot's shooter toward the hub and sets omega accordingly. */
  public void lockRotationOnHub() {
    Pose2d hubPose = FieldConstants.getHubPose().toPose2d();

    Translation2d shooterTranslation = new Translation2d(
        ShooterConstants.kShooterOffset.getX(),
        ShooterConstants.kShooterOffset.getY()).rotateBy(getPose().getRotation());

    Translation2d shooterFieldPos = getPose().getTranslation().plus(shooterTranslation);
    Translation2d toHub = hubPose.getTranslation().minus(shooterFieldPos);

    double desiredDelta = MathUtil.angleModulus(Math.atan2(toHub.getY(), toHub.getX())
        - ShooterConstants.kShooterOffset.getRotation().toRotation2d().getRadians());

    omega = feedbackController.calculate(getHeading().getRadians(), desiredDelta);
    if (feedbackController.atSetpoint()) {
      omega = 0;
    }
  }

  /** Returns true if the robot's heading is locked on the hub within tolerance. */
  public boolean isAtHeading() {
    return feedbackController.atSetpoint();
  }

  /** Stops the heading lock-on, clearing any commanded rotation. */
  public void endLockOn() {
    omega = 0;
  }

  /**
   * Sets the swerve module states.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets all drive encoders to position 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.resetYaw();
  }

  /**
   * Returns the heading of the robot from the pose estimator.
   *
   * @return The robot's heading as a Rotation2d.
   */
  public Rotation2d getHeading() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * Returns the turn rate of the robot in degrees per second.
   *
   * @return The turn rate.
   */
  public double getTurnRate() {
    return gyro.getAngularVel()[2].in(DegreesPerSecond) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
