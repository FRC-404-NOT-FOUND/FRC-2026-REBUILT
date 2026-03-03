// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax oneStageMotor;
  private final SparkMax twoStageMotor;
  private final RelativeEncoder oneStageEncoder;
  private final RelativeEncoder twoStageEncoder;
  private final SparkMaxConfig motorConfig;
  private final AlternateEncoderConfig encoderConfig;
  private final PIDController oneStagePID;
  private final PIDController twoStagePID;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    oneStageMotor = new SparkMax(ClimberConstants.oneStageCanID, MotorType.kBrushless);
    twoStageMotor = new SparkMax(ClimberConstants.twoStageCanID, MotorType.kBrushless);
    oneStageEncoder = oneStageMotor.getEncoder();
    twoStageEncoder = twoStageMotor.getAlternateEncoder();
    motorConfig = new SparkMaxConfig();
    encoderConfig = new AlternateEncoderConfig();

    motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    motorConfig.inverted(false);
    motorConfig.smartCurrentLimit(40);
    encoderConfig.positionConversionFactor(0); // Set this to the appropriate conversion factor for your encoder

    oneStageMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    twoStageMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    oneStagePID = new PIDController(0.1, 0, 0);
    twoStagePID = new PIDController(0.1, 0, 0);
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

  /**Drive motor to first stage in rotations */
  public void setOneStage(double position) {
    double output = oneStagePID.calculate(oneStageEncoder.getPosition(), position);
    oneStageMotor.set(output);
  }
  
  /**Drive motor to second stage in rotations */
  public void setTwoStage(double position) {
    double output = twoStagePID.calculate(twoStageEncoder.getPosition(), position);
    twoStageMotor.set(output);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
