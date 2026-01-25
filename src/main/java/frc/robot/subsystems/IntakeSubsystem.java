// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.IntakeContants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    motor = new SparkMax(IntakeContants.intakeCanID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    motorConfig.inverted(false);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Don't think we need to overcomplicate this tbh.
  public void spinIntake() {
    motor.set(0.3); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
