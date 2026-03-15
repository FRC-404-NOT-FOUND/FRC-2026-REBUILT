// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SpindexerSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;

  /** Creates a new SpindexerSubsystem. */
  public SpindexerSubsystem() {
    motor = new SparkMax(SpindexerConstants.spindexerCanID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    motorConfig.smartCurrentLimit(40);
    motorConfig.inverted(true);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Spin the spindexer. */
  public void startSpindexer() {
    motor.set(0.45);
  }

  /** Reverse spindexer for unjamming fuel. */
  public void reverseSpindexer() {
    motor.set(-0.45);
  }

  /** Stop spinning the spindexer. */
  public void stopSpindexer() {
    motor.set(0.0);
  }

  @Override
  public void periodic() {
    // Called once per scheduler run. Add telemetry or periodic checks here.
  }
}
