// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KickerConstants;

public class KickerSubsystem extends SubsystemBase {
  private final SparkFlex motor;
  private final RelativeEncoder motorEncoder;
  private final NetworkTable telemetryTable;

  /** Creates a new KickerSubsystem. */
  public KickerSubsystem() {
    motor = new SparkFlex(KickerConstants.kKickerCanId, MotorType.kBrushless);
    motorEncoder = motor.getEncoder();

    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(35).inverted(false);
    motorConfig.encoder.velocityConversionFactor(2 * Math.PI / 60); // no gearbox, rad/s
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    telemetryTable = NetworkTableInstance.getDefault().getTable("Kicker");
  }

  /** Spin the kicker wheel forward. */
  public void startKicker() {
    motor.set(KickerConstants.kForwardSpeed);
  }

  /** Stop the kicker wheel. */
  public void stopKicker() {
    motor.set(0);
  }

  /** Spin the kicker wheel in reverse to hold a note back. */
  public void reverseKicker() {
    motor.set(KickerConstants.kReverseSpeed);
  }

  @Override
  public void periodic() {
    telemetryTable.getEntry("CurrentVelocity").setDouble(motorEncoder.getVelocity());
  }
}
