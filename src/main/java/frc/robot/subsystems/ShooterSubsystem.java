// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShooterSubsystem extends SubsystemBase {
  // Empirically tuned velocities to use on the shooter at low and mid
  // distances from the hub
  public static final int lowVel = 300;
  public static final int midVel = 500;

  private final SparkFlex motorOne;
  private final RelativeEncoder motorOneEncoder;
  private final SparkFlexConfig motorOneConfig;

  private static final double shooterVelTolerance = 80; // plus or minus rad/sec

  private double kS = 0.15; // these both still need to be tuned
  private double kV = 0.0169; // Approximation
  private SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(kS, kV);

  private double kP = 0.02;
  private double kD = 0.003;
  private PIDController shooterFeedback = new PIDController(kP, 0, kD);

  private double kMaxVelocity = 710;
  private double kMaxAcceleration = 710;
  private TrapezoidProfile shooterProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration));
  private TrapezoidProfile.State shooterSetpoint = new TrapezoidProfile.State(0.0, 0.0);

  private final NetworkTable shooterVelocityTable;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    motorOne = new SparkFlex(ShooterConstants.shooterCanID, MotorType.kBrushless);
    motorOneEncoder = motorOne.getEncoder();
    motorOneConfig = new SparkFlexConfig();

    motorOneConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    motorOneConfig.inverted(false);
    motorOneConfig.smartCurrentLimit(50);
    motorOneConfig.encoder.velocityConversionFactor(2 * Math.PI / 60); // no gear box, rad/sec
    motorOne.configure(motorOneConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterVelocityTable = NetworkTableInstance.getDefault().getTable("Shooter");
  }

  /**
   * Sets the angular velocity of the shooter wheel.
   *
   * @param radPerSec Desired angular velocity of the shooter wheel in units of
   *                  radians per second.
   */
  public void setVelocity(double radPerSec) {
    double currentVelocity = motorOneEncoder.getVelocity();

    shooterSetpoint = shooterProfile.calculate(0.02,
        shooterSetpoint,
        new TrapezoidProfile.State(radPerSec, 0));
    double nextVelocity = shooterSetpoint.position;

    double voltage = shooterFeedforward.calculateWithVelocities(currentVelocity, nextVelocity)
        + shooterFeedback.calculate(currentVelocity, nextVelocity);
    motorOne.setVoltage(voltage);
  }

  /** Stop spinning the shooter wheel. */
  public void stopShooter() {
    motorOne.set(0);
    shooterSetpoint = new TrapezoidProfile.State(0.0, 0.0);
  }

  /**
   * Check if shooter wheel is close enough to target angular velocity.
   *
   * @param target Target angular velocity.
   * @return Whether or shooter is within tolerance.
   */
  public boolean shooterWithinTolerance(double target) {
    return Math.abs(target - motorOneEncoder.getVelocity()) <= shooterVelTolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterVelocityTable.getEntry("CurrentVelocity").setDouble(motorOneEncoder.getVelocity());
    shooterVelocityTable.getEntry("SetpointVelocity").setDouble(shooterSetpoint.position);
  }
}