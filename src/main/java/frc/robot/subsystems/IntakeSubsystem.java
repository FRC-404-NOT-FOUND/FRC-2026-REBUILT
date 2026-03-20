// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;
  private final SparkFlex vortexMotor;
  private final SparkFlexConfig vortexConfig;
  public boolean intakeIsSpinning;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    motor = new SparkMax(IntakeConstants.intakeCanID, MotorType.kBrushed);
    motorConfig = new SparkMaxConfig();
    vortexMotor = new SparkFlex(IntakeConstants.vortexCanID, MotorType.kBrushless);
    vortexConfig = new SparkFlexConfig();

    motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    vortexConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    motorConfig.inverted(true);
    vortexConfig.inverted(true); // placeholder
    motorConfig.smartCurrentLimit(35);
    vortexConfig.smartCurrentLimit(35);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    vortexMotor.configure(vortexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Spin the intake. */
  public void spinIntake() {
    motor.set(1);
    vortexMotor.set(1);
    intakeIsSpinning = true;
  }

  /** Spin the intake in reverse for unjamming fuel. */
  public void reverseIntake() {
    motor.set(-0.85);
    vortexMotor.set(-0.85);
  }

  /** Stop spinning the intake. */
  public void stopIntake() {
    motor.set(0);
    vortexMotor.set(0);
    intakeIsSpinning = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (motor.getOutputCurrent() > 30) {
      SmartDashboard.putBoolean("Intake/Possible jam", true);
    } else {
      SmartDashboard.putBoolean("Intake/Possible jam", false);
    }
  }
}