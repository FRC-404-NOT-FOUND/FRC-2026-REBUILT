// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkFlex motorOne;
  private final SparkFlex motorTwo;
  private final RelativeEncoder motorOneEncoder;
  private final RelativeEncoder motorTwoEncoder;

  private final SimpleMotorFeedforward shooterFeedforward =
      new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);
  private final PIDController shooterFeedback = new PIDController(ShooterConstants.kP, 0, 0);
  private final TrapezoidProfile shooterProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(ShooterConstants.kMaxVelocity, ShooterConstants.kMaxAcceleration));

  private TrapezoidProfile.State shooterSetpoint = new TrapezoidProfile.State(0.0, 0.0);
  private double feedback = 0.0;

  private final NetworkTable telemetryTable;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    motorOne = new SparkFlex(ShooterConstants.kShooter1CanId, MotorType.kBrushless);
    motorTwo = new SparkFlex(ShooterConstants.kShooter2CanId, MotorType.kBrushless);
    motorOneEncoder = motorOne.getEncoder();
    motorTwoEncoder = motorTwo.getEncoder();

    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kCoast).inverted(false).smartCurrentLimit(50);
    motorConfig.encoder.velocityConversionFactor(2 * Math.PI / 60); // no gearbox, rad/s
    motorOne.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorConfig.follow(motorOne, true);
    motorTwo.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterFeedback.setTolerance(ShooterConstants.kFeedbackPidTolerance);

    telemetryTable = NetworkTableInstance.getDefault().getTable("Shooter");
  }

  /**
   * Sets the angular velocity of the shooter wheel.
   *
   * @param radPerSec Desired angular velocity in radians per second.
   */
  public void setVelocity(double radPerSec) {
    double currentVelocity = motorOneEncoder.getVelocity();

    shooterSetpoint = shooterProfile.calculate(0.02,
        shooterSetpoint,
        new TrapezoidProfile.State(radPerSec, 0));
    double nextVelocity = shooterSetpoint.position;

    double feedforward = shooterFeedforward.calculateWithVelocities(currentVelocity, nextVelocity);
    feedback = shooterFeedback.calculate(currentVelocity, nextVelocity);
    if (shooterFeedback.atSetpoint()) {
      feedback = 0.0;
      shooterFeedback.reset(); // prevents derivative spike
    }
    motorOne.setVoltage(feedforward + feedback);
  }

  /** Stop the shooter wheel. */
  public void stopShooter() {
    motorOne.set(0);
    shooterSetpoint = new TrapezoidProfile.State(0.0, 0.0);
  }

  /**
   * Returns true if the shooter wheel is within tolerance of the target velocity.
   *
   * @param target Target angular velocity in radians per second.
   */
  public boolean shooterWithinTolerance(double target) {
    return Math.abs(target - motorOneEncoder.getVelocity()) <= ShooterConstants.kVelocityTolerance;
  }

  @Override
  public void periodic() {
    double velocity = motorOneEncoder.getVelocity();
    telemetryTable.getEntry("MotorOneVelocity").setDouble(Math.floor(velocity * 100) / 100);
    telemetryTable.getEntry("MotorTwoVelocity").setDouble(motorTwoEncoder.getVelocity());
    telemetryTable.getEntry("SetpointVelocity").setDouble(Math.floor(shooterSetpoint.position * 100) / 100);
    telemetryTable.getEntry("VelocityError").setDouble(Math.floor((shooterSetpoint.position - velocity) * 100) / 100);
  }
}
