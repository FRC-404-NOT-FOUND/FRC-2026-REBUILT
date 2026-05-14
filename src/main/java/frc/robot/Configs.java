package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
  public static final class MAXSwerve {
    public static final SparkMaxConfig kDrivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kTurningConfig = new SparkMaxConfig();

    static {
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
          / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double nominalVoltage = 12.0;
      double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

      kDrivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(45);
      kDrivingConfig.encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      kDrivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(0.04, 0, 0)
          .outputRange(-1, 1)
          .feedForward.kV(drivingVelocityFeedForward);

      kTurningConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(35);
      kTurningConfig.absoluteEncoder
          // Output shaft rotates opposite to the steering motor in the MAXSwerve module
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0) // radians per second
          .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);
      kTurningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }
}
