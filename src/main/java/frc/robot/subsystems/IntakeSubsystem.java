// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private final SparkFlex vortexMotor;
  private boolean spinning;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    motor = new SparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushed);
    vortexMotor = new SparkFlex(IntakeConstants.kVortexCanId, MotorType.kBrushless);

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(35).inverted(true);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig vortexConfig = new SparkFlexConfig();
    vortexConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(35).inverted(true);
    vortexMotor.configure(vortexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Spin the intake. */
  public void spinIntake() {
    motor.set(IntakeConstants.kForwardSpeed);
    vortexMotor.set(IntakeConstants.kForwardSpeed);
    spinning = true;
  }

  /** Spin the intake in reverse for unjamming. */
  public void reverseIntake() {
    motor.set(IntakeConstants.kReverseSpeed);
    vortexMotor.set(IntakeConstants.kReverseSpeed);
  }

  /** Stop the intake. */
  public void stopIntake() {
    motor.set(0);
    vortexMotor.set(0);
    spinning = false;
  }

  /** Returns true if the intake is actively spinning forward. */
  public boolean isSpinning() {
    return spinning;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake/Possible jam", motor.getOutputCurrent() > IntakeConstants.kJamCurrentThreshold);
  }
}
