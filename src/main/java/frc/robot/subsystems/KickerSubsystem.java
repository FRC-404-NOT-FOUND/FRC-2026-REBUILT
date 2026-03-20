// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KickerConstants;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

public class KickerSubsystem extends SubsystemBase {
  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;
  private final RelativeEncoder motorEncoder;

  private final NetworkTable kickerVelocityTable;

  /** Creates a new KickerSubsytem. */
  public KickerSubsystem() {
    motor = new SparkFlex(KickerConstants.kickerCanID, MotorType.kBrushless);
    motorConfig = new SparkFlexConfig();
    motorEncoder = motor.getEncoder();

    motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    motorConfig.inverted(false);
    motorConfig.smartCurrentLimit(35);
    motorConfig.encoder.velocityConversionFactor(2 * Math.PI / 60); // no gear box, rad/sec
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    kickerVelocityTable = NetworkTableInstance.getDefault().getTable("Kicker");
  }

  /** Spin the kicker wheel. */
  public void startKicker() {
    motor.set(0.3);
  }

  /** Stop spinning the kicker wheel. */
  public void stopKicker() {
    motor.set(0);
  }

  public void reverseKicker() {
    motor.set(-0.35);
  }

  @Override
  public void periodic() {
    // Called once per scheduler run. Add telemetry or periodic checks here.
    kickerVelocityTable.getEntry("CurrentVelocity").setDouble(motorEncoder.getVelocity());
  }
}
