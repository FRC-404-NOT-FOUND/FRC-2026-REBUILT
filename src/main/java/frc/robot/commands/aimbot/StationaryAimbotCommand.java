// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.aimbot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StationaryAimbotCommand extends Command {
  private final DriveSubsystem drive;
  private final ShooterSubsystem shooter;
  private final KickerSubsystem kicker;
  private final SpindexerSubsystem spindexer;

  /**
   * Creates a new StationaryAimbotCommand.
   * 
   * @param drive
   * @param shooter
   * @param kicker
   * @param spindexer
   */
  public StationaryAimbotCommand(DriveSubsystem drive, ShooterSubsystem shooter, KickerSubsystem kicker,
      SpindexerSubsystem spindexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.shooter = shooter;
    this.kicker = kicker;
    this.spindexer = spindexer;
    addRequirements(drive, shooter, kicker, spindexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose3d hubPose = FieldConstants.hubPose;
    Pose3d shooterPose = new Pose3d(drive.getPose()).plus(ShooterConstants.shooterOffset);
    Pose3d relativePose = hubPose.relativeTo(shooterPose);

    // Projectile trajectory math
    double theta = ShooterConstants.theta; // theta of shooter wheel from horizontal, radians
    double rS = ShooterConstants.radius; // radius of shooter wheel
    double g = ShooterConstants.g; // acceleration of gravity
    double x = Math.hypot(relativePose.getX(), relativePose.getY()) + StationaryAimbotCommandData.getOffsetMeters(); // horizontal distance from robot to hub
    double y = relativePose.getZ(); // vertical distance from robot to hub
    double k = 0.35; // efficiency, "fudge factor"

    drive.lockRotationOnHub();
    drive.drive(0, 0, 0, true);

    // Don't divide by zero
    if (Math.tan(theta) * x <= y) {
        return;
    }

    double vB = (x / Math.cos(theta)) * Math.sqrt(g / (2 * (Math.tan(theta) * x - y))); // required ball velocity
    double vS = vB / (k * rS); // required shooter angular velocity, rad/sec

    shooter.setVelocity(vS);

    // Feed when at target velocity
    if (shooter.shooterWithinTolerance(vS)) {
      kicker.startKicker();
      spindexer.startSpindexer();
    } else {
      kicker.stopKicker();
      spindexer.stopSpindexer();
    }

        SmartDashboard.putNumber("Aimbot/Horizontal Distance (m)", x);
    SmartDashboard.putNumber("Solved rad/s", vS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spindexer.stopSpindexer();
    kicker.stopKicker();
    shooter.stopShooter();
    drive.endLockOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
