// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

import frc.robot.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final SparkMax motorOne;
  private final RelativeEncoder motorOneEncoder;
  private final SparkMaxConfig motorOneConfig;

  //empirically tuned velocities to use on the shooter at low, mid, and high distances from the hub
  public static final int lowVel = 0;
  public static final int midVel = 0;
  public static final int highVel = 0;

  private double kS = 0;
  private double kV =  1 / Units.rotationsPerMinuteToRadiansPerSecond(565); // RPM per volt listed as 565 on neo vortex spec sheet
  private SimpleMotorFeedforward shooter_feedforward = new SimpleMotorFeedforward(kS, kV);
  
  private double kMaxVelocity = 0;
  private double kMaxAcceleration = 0;
  private TrapezoidProfile shooter_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration));
  private TrapezoidProfile.State shooterSetpoint =
    new TrapezoidProfile.State(0.0, 0.0);

  public ShooterSubsystem() {
    motorOne = new SparkMax(Constants.ShooterContants.shooterOneCanID, MotorType.kBrushless);
    motorOneEncoder = motorOne.getEncoder();
    motorOneConfig = new SparkMaxConfig();

    motorOneConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    motorOneConfig.inverted(false);
    motorOneConfig.encoder.velocityConversionFactor(2 * Math.PI / 60); // no gear box
  }

  public void setVoltage(double voltage) { // actually not sure if this will even be used
    motorOne.setVoltage(voltage);
  }

  public void setVelocity(double radPerSec) {
    double currentVelocity = motorOneEncoder.getVelocity();

    shooterSetpoint = shooter_profile.calculate(0.02
    , shooterSetpoint
    , new TrapezoidProfile.State(radPerSec, 0));
    double nextVelocity = shooterSetpoint.velocity;

    motorOne.setVoltage(shooter_feedforward.calculateWithVelocities(currentVelocity, nextVelocity));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
