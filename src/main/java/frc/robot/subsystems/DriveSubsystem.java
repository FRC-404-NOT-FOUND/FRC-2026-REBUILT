// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import org.photonvision.EstimatedRobotPose;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import com.studica.frc.Navx;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final Navx m_gyro = new Navx(DriveConstants.kGyroPort, 100);
  private double gyroAngle;

  // Pose logging
  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
      .getStructTopic("MyPose", Pose2d.struct).publish();

  // PID control for auto lock on
  public PIDController mFeedbackController = new PIDController(1, 0, 0); // TUNE THIS.
  private double omega;

  // Pose estimation
  SwerveDrivePoseEstimator mPoseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getYaw().in(Degrees)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }, new Pose2d());

  /**
   * Adds vision measurement to kalman filter.
   * 
   * @param visionPose Estimated pose from april tag.
   * @param stdDevs    How much we trust the given pose.
   */
  public void addVisionMeasurement(EstimatedRobotPose visionPose, Matrix<N3, N1> stdDevs) {
    mPoseEstimator.addVisionMeasurement(visionPose.estimatedPose.toPose2d(), visionPose.timestampSeconds,
        stdDevs);
  }

  

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_gyro.enableOptionalMessages(true, true, true, false, false, false, false, false, false, false);

    mFeedbackController.enableContinuousInput(-Math.PI, Math.PI);
    mFeedbackController.setTolerance(Math.toRadians(5));

    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    // Configure AutoBuilder last
    // Stolen from PathPlanner docs
    RobotConfig config = PathPlannerConstants.config;
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> pathplannerRelativeDrive(speeds), // Method that will drive the robot given ROBOT
                                                                    // RELATIVE ChassisSpeeds. Also optionally outputs
                                                                    // individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(1.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    gyroAngle = m_gyro.getYaw().in(Degrees) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    mPoseEstimator.update(
        Rotation2d.fromDegrees(gyroAngle),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    publisher.set(getPose());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return mPoseEstimator.getEstimatedPosition();
  }

  /**
   * Resets to the specified pose.
   *
   * @param pose The pose to set.
   */
  public void resetPose(Pose2d pose) {
    mPoseEstimator.resetPosition(
        getHeading(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /** Returns robot relative ChassisSpeeds */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  /**
   * Robot relative drive for use with PathPlanner.
   * 
   * @param speeds ChassisSpeeds to set modules to.
   */
  public void pathplannerRelativeDrive(ChassisSpeeds speeds) {
    var states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(states);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Use always blue origin
    var alliance = DriverStation.getAlliance();
    var alwaysBlueInvert = 1;
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      alwaysBlueInvert = -1;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = (omega != 0) ? omega : (rot * DriveConstants.kMaxAngularSpeed);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered * alwaysBlueInvert,
                ySpeedDelivered * alwaysBlueInvert, rotDelivered,
                (getHeading()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Locks the rotation of the robot to face the hub.
   */
  public void lockRotationOnHub() {
    Pose2d hubPose = FieldConstants.getHubPose().toPose2d();

    // Rotate shooter's X/Y offset into field frame
    Translation2d shooterTranslation = new Translation2d(
        ShooterConstants.shooterOffset.getX(),
        ShooterConstants.shooterOffset.getY()).rotateBy(getPose().getRotation());

    // Shooter's actual position on the field
    Translation2d shooterFieldPos = getPose().getTranslation().plus(shooterTranslation);

    // Vector from shooter to hub
    Translation2d toHub = hubPose.getTranslation().minus(shooterFieldPos);

    // Angle from shooter to hub, minus shooter's yaw offset so the barrel faces the
    // hub
    double desiredDelta = MathUtil.angleModulus(Math.atan2(toHub.getY(), toHub.getX())
        - ShooterConstants.shooterOffset.getRotation().toRotation2d().getRadians());

    omega = mFeedbackController.calculate(getHeading().getRadians(), desiredDelta);
    if (mFeedbackController.atSetpoint()) {
      omega = 0;
    }
  }

  public void endLockOn() {
    omega = 0;
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.resetYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return mPoseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getAngularVel()[2].in(DegreesPerSecond) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
