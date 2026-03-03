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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax oneStageMotor;
  private final SparkMax twoStageMotor;
  private final RelativeEncoder oneStageEncoder;
  private final RelativeEncoder twoStageEncoder;
  private final PIDController oneStagePID;
  private final PIDController twoStagePID;
  private static final double POSITION_CONVERSION_FACTOR = 1.0; // We should most likely change this so it doesn't blow up PID
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    oneStageMotor = new SparkMax(ClimberConstants.oneStageCanID, MotorType.kBrushless);
    twoStageMotor = new SparkMax(ClimberConstants.twoStageCanID, MotorType.kBrushless);

    AlternateEncoderConfig encoderConfig = new AlternateEncoderConfig();
    encoderConfig.positionConversionFactor(POSITION_CONVERSION_FACTOR);

    SparkMaxConfig oneStageConfig = new SparkMaxConfig();
    oneStageConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    oneStageConfig.inverted(false);
    oneStageConfig.smartCurrentLimit(40);
    oneStageConfig.apply(encoderConfig); 

    SparkMaxConfig twoStageConfig = new SparkMaxConfig();
    twoStageConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    twoStageConfig.inverted(false);
    twoStageConfig.smartCurrentLimit(40);
    twoStageConfig.apply(encoderConfig);

    oneStageMotor.configure(oneStageConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    twoStageMotor.configure(twoStageConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    oneStageEncoder = oneStageMotor.getAlternateEncoder();
    twoStageEncoder = twoStageMotor.getAlternateEncoder();
    
    oneStagePID = new PIDController(0.1, 0, 0);
    twoStagePID = new PIDController(0.1, 0, 0);

    oneStagePID.setTolerance(0.05);
    twoStagePID.setTolerance(0.05);
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
    oneStageMotor.set(MathUtil.clamp(output, -1.0, 1.0));
  }
  
  /**Drive motor to second stage in rotations */
  public void setTwoStage(double position) {
    double output = twoStagePID.calculate(twoStageEncoder.getPosition(), position);
    twoStageMotor.set(MathUtil.clamp(output, -1.0, 1.0));
  }
  
      public boolean oneStageAtSetpoint() {
        return oneStagePID.atSetpoint();
    }

    public boolean twoStageAtSetpoint() {
        return twoStagePID.atSetpoint();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
