// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.studica.frc.Navx;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static).
 * Do not put anything functional in this class.
 */
public final class Constants {
  public static final class DriveConstants {
    // Allowed maximum speeds (not hardware maximums)
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // 27x27 chassis minus 3 in wheels gives 24 in between wheel centers
    public static final double kTrackWidth = Units.inchesToMeters(24);
    public static final double kWheelBase = Units.inchesToMeters(24);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of each module relative to the chassis (radians)
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 11;
    public static final int kRearLeftTurningCanId = 33;
    public static final int kFrontRightTurningCanId = 22;
    public static final int kRearRightTurningCanId = 44;

    public static final Navx.Port kGyroPort = Navx.Port.kUSB1;
    public static final boolean kGyroReversed = false;

    // PathPlanner path-following PID gains
    public static final double kPathplannerTranslationP = 2.75;
    public static final double kPathplannerRotationP = 2.0;

    // Heading lock-on PID gains
    public static final double kHeadingP = 1.5;
    public static final double kHeadingToleranceDegrees = 0.5;
  }

  public static final class ModuleConstants {
    public static final int kDrivingMotorPinionTeeth = 14;

    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on wheel bevel, 22 teeth on first-stage spur, 15 teeth on bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class PathPlannerConstants {
    public static RobotConfig config;

    static {
      try {
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        DriverStation.reportError("Failed to load PathPlanner robot config: " + e.getMessage(), e.getStackTrace());
      }
    }
  }

  public static final class ShooterConstants {
    // CAN IDs
    public static final int kShooter1CanId = 5;
    public static final int kShooter2CanId = 14;

    // Physical properties
    public static final double kTheta = Math.toRadians(74); // shooter angle off horizontal
    public static final double kGravity = 9.81; // m/s^2
    public static final double kWheelRadius = Units.inchesToMeters(2); // shooter wheel radius

    // Shooter position offset from robot center
    public static final Transform3d kShooterOffset = new Transform3d(
        new Translation3d(-0.254, -0.1778, 0.4318),
        new Rotation3d(0, 0, Math.PI / 2));

    // Feedforward gains (kS in volts, kV in V*s/rad)
    public static final double kS = 0.16;
    public static final double kV = 0.017;

    // Feedback gain
    public static final double kP = 0.032;

    // Trapezoid profile constraints (rad/s and rad/s^2)
    public static final double kMaxVelocity = 710;
    public static final double kMaxAcceleration = 710;

    // Tolerance for "shooter is ready to fire" check (rad/s)
    public static final double kVelocityTolerance = 50;

    // Tolerance for PID oscillation damping (rad/s)
    public static final double kFeedbackPidTolerance = 5;

    // Named preset velocities (rad/s)
    public static final double kLowVelocity = 300;
    public static final double kMidVelocity = 500;

    // Distance (m) -> velocity (rad/s) lookup table for aimbot
    public static final InterpolatingDoubleTreeMap kVelocityMap = new InterpolatingDoubleTreeMap();
    static {
      kVelocityMap.put(0.0, 0.0);
    }
  }

  public static final class IntakeConstants {
    public static final int kIntakeCanId = 8;
    public static final int kVortexCanId = 13;

    public static final double kForwardSpeed = 1.0;
    public static final double kReverseSpeed = -0.85;
    public static final double kJamCurrentThreshold = 30; // amps
  }

  public static final class SpindexerConstants {
    public static final int kSpindexerCanId = 7;

    public static final double kForwardSpeed = 0.6;
    public static final double kReverseSpeed = -0.5;
    public static final double kJamCurrentThreshold = 35; // amps
  }

  public static final class KickerConstants {
    public static final int kKickerCanId = 6;

    public static final double kForwardSpeed = 0.3;
    public static final double kReverseSpeed = -0.35;
  }

  public static final class VisionConstants {
    public static final String kCam1Name = "Shoot Camera";
    public static final String kCam2Name = "Intake Camera";

    // Camera transforms from robot center
    public static final Transform3d kRobotToCam1 = new Transform3d(
        new Translation3d(-0.1285875, 0.3175, 0.51435),
        new Rotation3d(0, 0, Math.PI / 2));
    public static final Transform3d kRobotToCam2 = new Transform3d(
        new Translation3d(0.3175, -0.1793875, 0.36195),
        new Rotation3d(0, 0, 0));

    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    // Pose filtering thresholds
    public static final double kMaxAmbiguity = 0.2;
    public static final double kMaxZError = 0.75; // meters

    // Baseline standard deviations at 1 m distance, 1 tag
    public static final double kLinearStdDevBaseline = 0.5; // meters
    public static final double kAngularStdDevBaseline = 0.5; // radians

    // Per-camera trust multipliers
    public static final double[] kCameraStdDevFactors = new double[] {
        1.0, // Camera 1
        1.0  // Camera 2
    };

    // Multi-tag solve trust multipliers (more stable than full 3D solve)
    public static final double kLinearStdDevMultitagFactor = 0.5;
    public static final double kAngularStdDevMultitagFactor = 0.5;
  }

  public static final class FieldConstants {
    private static final Pose3d kBlueHubPose = new Pose3d(
        Units.inchesToMeters(181.56), Units.inchesToMeters(158.32),
        Units.inchesToMeters(72), new Rotation3d());
    private static final Pose3d kRedHubPose = new Pose3d(
        Units.inchesToMeters(468.56), Units.inchesToMeters(158.32),
        Units.inchesToMeters(72), new Rotation3d());

    /** Returns the hub pose for the current alliance, defaulting to blue. */
    public static Pose3d getHubPose() {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        return kRedHubPose;
      }
      return kBlueHubPose;
    }
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
