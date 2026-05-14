// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;

public class SpindexerSubsystem extends SubsystemBase {
  private final SparkMax motor;

  /** Creates a new SpindexerSubsystem. */
  public SpindexerSubsystem() {
    motor = new SparkMax(SpindexerConstants.kSpindexerCanId, MotorType.kBrushless);

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(true);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Spin the spindexer. */
  public void startSpindexer() {
    motor.set(SpindexerConstants.kForwardSpeed);
  }

  /** Reverse the spindexer for unjamming. */
  public void reverseSpindexer() {
    motor.set(SpindexerConstants.kReverseSpeed);
  }

  /** Stop the spindexer. */
  public void stopSpindexer() {
    motor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Spindexer/Possible jam",
        motor.getOutputCurrent() > SpindexerConstants.kJamCurrentThreshold);
  }
}
