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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    // 27x27 minus 3in wheels
    public static final double kTrackWidth = Units.inchesToMeters(24);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
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
    public static final boolean kGyroReversed = false; // if gyro is giving reversed values, change this to true
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class PathPlannerConstants {
    public static RobotConfig config;

    static {
      try {
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
      }
    }
  }

  public static final class ShooterConstants {
    public static final double theta = Math.toRadians(74); // angle of shooter off horizontal
    public static final double g = 9.81; // gravity
    public static final double radius = Units.inchesToMeters(2); // shooter wheel

    public static final int shooterCanID = 5;

    public static final Transform3d shooterOffset = new Transform3d(new Translation3d(-0.254, -0.1778, 0.4318),
        new Rotation3d(0, 0, Math.PI / 2));
  }

  public static final class IntakeConstants {
    public static final int intakeCanID = 8;
    public static final int vortexCanID = 13;
  }

  public static final class SpindexerConstants {
    public static final int spindexerCanID = 7;
  }

  public static final class KickerConstants {
    public static final int kickerCanID = 6;
  }

  public static final class ClimberConstants {
    public static final int oneStageCanID = 10;
    public static final int twoStageCanID = 9;
  }

  public static final class VisionConstants {
    // Camera names from PV
    public static final String cam1 = "ClimbShoot Camera";
    public static final String cam2 = "Intake Camera";

    // Camera offsets from robot center
    public static final Transform3d kRobotToCamOne = new Transform3d(new Translation3d(-0.127, 0.3175, 0.51435),
        new Rotation3d(0, 0, Math.PI / 2));
    public static final Transform3d kRobotToCamTwo = new Transform3d(new Translation3d(0.3175, -0.206375, 0.365125),
        new Rotation3d(0, 0, 0));

    // April tags
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 1
        1.0 // Camera 2
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMultitagFactor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMultitagFactor = 0.5;
  }

  public static final class FieldConstants {
    // Hub positions hardcoded - do NOT read alliance in a static block
    private static final Pose3d kBlueHubPose = new Pose3d(
        Units.inchesToMeters(181.56), Units.inchesToMeters(158.32),
        Units.inchesToMeters(72), new Rotation3d());
    private static final Pose3d kRedHubPose = new Pose3d(
        Units.inchesToMeters(468.56), Units.inchesToMeters(158.32),
        Units.inchesToMeters(72), new Rotation3d());

    /** Always returns the correct hub pose for the current alliance. */
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
