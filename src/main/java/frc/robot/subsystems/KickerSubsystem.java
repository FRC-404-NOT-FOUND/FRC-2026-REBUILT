// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KickerConstants;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

public class KickerSubsystem extends SubsystemBase {
  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;

  /** Creates a new KickerSubsytem. */
  public KickerSubsystem() {
    motor = new SparkFlex(KickerConstants.kickerCanID, MotorType.kBrushless);
    motorConfig = new SparkFlexConfig();

    motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    motorConfig.inverted(false);
    motorConfig.smartCurrentLimit(40);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    motor.set(-0.2);
  }

  @Override
  public void periodic() {
    // Called once per scheduler run. Add telemetry or periodic checks here.
  }
}
