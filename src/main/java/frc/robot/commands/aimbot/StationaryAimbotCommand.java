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
import edu.wpi.first.wpilibj.DriverStation;
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
  public void execute() {
    Pose3d hubPose = FieldConstants.getHubPose();
    Pose3d shooterPose = new Pose3d(drive.getPose()).plus(ShooterConstants.shooterOffset);
    Pose3d relativePose = hubPose.relativeTo(shooterPose);

    double theta = ShooterConstants.theta;
    double rS = ShooterConstants.radius;
    double g = ShooterConstants.g;
    double x = Math.hypot(relativePose.getX(), relativePose.getY()) + StationaryAimbotCommandData.getOffsetMeters();
    double y = relativePose.getZ();
    double k = 0.3;

    // --- Logging: pose & geometry ---
    SmartDashboard.putNumber("Aimbot/Shooter Pose X (m)", shooterPose.getX());
    SmartDashboard.putNumber("Aimbot/Shooter Pose Y (m)", shooterPose.getY());
    SmartDashboard.putNumber("Aimbot/Shooter Pose Z (m)", shooterPose.getZ());
    SmartDashboard.putNumber("Aimbot/Hub Pose Z (m)", hubPose.getZ());
    SmartDashboard.putNumber("Aimbot/Relative X (m)", relativePose.getX());
    SmartDashboard.putNumber("Aimbot/Relative Y (m)", relativePose.getY());
    SmartDashboard.putNumber("Aimbot/Vertical Distance Y (m)", y);
    SmartDashboard.putNumber("Aimbot/Horizontal Distance X (m)", x);
    SmartDashboard.putNumber("Aimbot/Fudge Offset (m)", StationaryAimbotCommandData.getOffsetMeters());

    drive.lockRotationOnHub();
    drive.drive(0, 0, 0, true);

    double tanThetaX = Math.tan(theta) * x;

    if (tanThetaX <= y) {
      shooter.stopShooter();
      kicker.stopKicker();
      spindexer.stopSpindexer();
      SmartDashboard.putBoolean("Aimbot/Shot Possible", false);
      return;
    }
    SmartDashboard.putBoolean("Aimbot/Shot Possible", true);

    double vB = (x / Math.cos(theta)) * Math.sqrt(g / (2 * (tanThetaX - y)));
    double vS = vB / (k * rS);

    // --- Logging: solver outputs ---
    SmartDashboard.putNumber("Aimbot/Required Ball Velocity (m-s)", vB);
    SmartDashboard.putNumber("Aimbot/Required Shooter (rad-s)", vS);
    SmartDashboard.putNumber("Aimbot/Required Shooter (RPM)", vS * 60 / (2 * Math.PI));
    SmartDashboard.putString("Aimbot/Alliance",
        DriverStation.getAlliance().map(Object::toString).orElse("UNKNOWN"));

    shooter.setVelocity(vS);

    SmartDashboard.putBoolean("Aimbot/Shooter At Speed", shooter.shooterWithinTolerance(vS));
    SmartDashboard.putBoolean("Aimbot/Drive At Heading", drive.mFeedbackController.atSetpoint());

    if (shooter.shooterWithinTolerance(vS)) {
      kicker.startKicker();
      spindexer.startSpindexer();
      SmartDashboard.putBoolean("Aimbot/Feeding", true);
    } else {
      kicker.stopKicker();
      spindexer.stopSpindexer();
      SmartDashboard.putBoolean("Aimbot/Feeding", false);
    }
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
