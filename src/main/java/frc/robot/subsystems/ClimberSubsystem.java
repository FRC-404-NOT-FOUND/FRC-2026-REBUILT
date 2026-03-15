// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax oneStageMotor;
  private final SparkMax twoStageMotor;
  private final SparkMaxConfig motorConfig;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    oneStageMotor = new SparkMax(ClimberConstants.oneStageCanID, MotorType.kBrushless);
    twoStageMotor = new SparkMax(ClimberConstants.twoStageCanID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    motorConfig.inverted(false);
    motorConfig.smartCurrentLimit(5);
    motorConfig.encoder.positionConversionFactor(1); // Motor rotations to meters

    oneStageMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorConfig.inverted(true);
    twoStageMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setCoastMode(SparkMax motor) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(SparkBaseConfig.IdleMode.kCoast);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setBrakeMode(SparkMax motor) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void oneStageSet(double percentage) {
    oneStageMotor.set(percentage);
  }

  public void twoStageSet(double percentage) {
    twoStageMotor.set(percentage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
