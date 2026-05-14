// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.aimbot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

public class StationaryAimbotCommand extends Command {
  private final DriveSubsystem drive;
  private final ShooterSubsystem shooter;
  private final KickerSubsystem kicker;
  private final SpindexerSubsystem spindexer;
  private final IntakeSubsystem intake;
  private final boolean alwaysKick;
  private final boolean useLookupTable;
  private final InterpolatingDoubleTreeMap velocityTable = ShooterConstants.kVelocityMap;

  /**
   * Creates a new StationaryAimbotCommand.
   *
   * @param drive          Drive subsystem for heading lock.
   * @param shooter        Shooter subsystem.
   * @param kicker         Kicker subsystem.
   * @param spindexer      Spindexer subsystem.
   * @param intake         Intake subsystem.
   * @param alwaysKick     If true, feed regardless of shooter speed.
   * @param useLookupTable If true, use distance->velocity map instead of ballistic calculation.
   */
  public StationaryAimbotCommand(DriveSubsystem drive, ShooterSubsystem shooter, KickerSubsystem kicker,
      SpindexerSubsystem spindexer, IntakeSubsystem intake, boolean alwaysKick, boolean useLookupTable) {
    this.drive = drive;
    this.shooter = shooter;
    this.kicker = kicker;
    this.spindexer = spindexer;
    this.intake = intake;
    this.alwaysKick = alwaysKick;
    this.useLookupTable = useLookupTable;
    addRequirements(drive, shooter, kicker, spindexer, intake);
  }

  @Override
  public void initialize() {
    // Reverse kicker to hold note back until shooter reaches speed
    kicker.reverseKicker();
    intake.spinIntake();
  }

  @Override
  public void execute() {
    Pose3d hubPose = FieldConstants.getHubPose();
    Pose3d shooterPose = new Pose3d(drive.getPose()).plus(ShooterConstants.kShooterOffset);
    Pose3d relativePose = hubPose.relativeTo(shooterPose);

    double theta = ShooterConstants.kTheta;
    double rS = ShooterConstants.kWheelRadius;
    double g = ShooterConstants.kGravity;
    double x = Math.hypot(relativePose.getX(), relativePose.getY()) + StationaryAimbotCommandData.getOffsetMeters();
    double y = relativePose.getZ();
    double k = 0.225; // empirical spin-to-velocity ratio

    SmartDashboard.putNumber("Aimbot/Horizontal Distance X (m)", Math.floor(x * 100) / 100);
    SmartDashboard.putNumber("Aimbot/Fudge Offset (m)", StationaryAimbotCommandData.getOffsetMeters());

    drive.lockRotationOnHub();
    drive.drive(0, 0, 0, true);

    double tanThetaX = Math.tan(theta) * x;

    // Don't shoot if the ballistic trajectory is physically impossible
    if (tanThetaX <= y) {
      shooter.stopShooter();
      kicker.stopKicker();
      spindexer.stopSpindexer();
      SmartDashboard.putBoolean("Aimbot/Shot Possible", false);
      return;
    }
    SmartDashboard.putBoolean("Aimbot/Shot Possible", true);

    double vB = (x / Math.cos(theta)) * Math.sqrt(g / (2 * (tanThetaX - y)));
    double vS = useLookupTable ? velocityTable.get(x) : vB / (k * rS);

    shooter.setVelocity(vS);

    SmartDashboard.putBoolean("Aimbot/Shooter At Speed", shooter.shooterWithinTolerance(vS));
    SmartDashboard.putBoolean("Aimbot/Drive At Heading", drive.isAtHeading());

    if (shooter.shooterWithinTolerance(vS) || alwaysKick) {
      kicker.startKicker();
      spindexer.startSpindexer();
      SmartDashboard.putBoolean("Aimbot/Feeding", true);
    } else {
      // Kicker intentionally left reversing (from initialize) to hold note back until shooter is at speed
      spindexer.stopSpindexer();
      SmartDashboard.putBoolean("Aimbot/Feeding", false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    spindexer.stopSpindexer();
    kicker.stopKicker();
    shooter.stopShooter();
    intake.stopIntake();
    drive.endLockOn();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
