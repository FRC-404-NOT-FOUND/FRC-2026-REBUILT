// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.aimbot.StationaryAimbotCommand;
import frc.robot.commands.aimbot.StationaryAimbotCommandData;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem drive = new DriveSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final SpindexerSubsystem spindexer = new SpindexerSubsystem();
  private final KickerSubsystem kicker = new KickerSubsystem();
  private final Vision vision = new Vision(drive);

  private final SlewRateLimiter xSlewRateLimiter;
  private final SlewRateLimiter ySlewRateLimiter;
  private final SlewRateLimiter rSlewRateLimiter;

  // Blending factor between cubic (a=1) and linear (a=0) drive response curve
  private final double a;

  private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("Start Intake", new InstantCommand(() -> intake.spinIntake()));
    NamedCommands.registerCommand("Stop Intake", new InstantCommand(() -> intake.stopIntake()));
    NamedCommands.registerCommand("Stationary Aimbot",
        new StationaryAimbotCommand(drive, shooter, kicker, spindexer, intake, false, false));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureButtonBindings();

    xSlewRateLimiter = new SlewRateLimiter(1.8); // seconds to reach max
    ySlewRateLimiter = new SlewRateLimiter(1.8);
    rSlewRateLimiter = new SlewRateLimiter(2.5);

    a = 1.0;

    drive.setDefaultCommand(
        new RunCommand(
            () -> drive.drive(
                -MathUtil.applyDeadband(
                    xSlewRateLimiter.calculate(
                        a * Math.pow(driverController.getLeftY(), 3) + (1 - a) * driverController.getLeftY()),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    ySlewRateLimiter.calculate(
                        a * Math.pow(driverController.getLeftX(), 3) + (1 - a) * driverController.getLeftX()),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    rSlewRateLimiter.calculate(
                        a * Math.pow(driverController.getRightX(), 3) + (1 - a) * driverController.getRightX()),
                    OIConstants.kDriveDeadband),
                true),
            drive));
  }

  private void configureButtonBindings() {
    // Y: reverse intake
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whileTrue(new StartEndCommand(
            () -> intake.reverseIntake(),
            () -> intake.stopIntake(),
            intake));

    // A: reverse all feed mechanisms (unjam)
    new JoystickButton(driverController, XboxController.Button.kA.value)
        .whileTrue(new StartEndCommand(
            () -> {
              spindexer.reverseSpindexer();
              kicker.reverseKicker();
              intake.reverseIntake();
            },
            () -> {
              spindexer.stopSpindexer();
              kicker.stopKicker();
              intake.stopIntake();
            },
            spindexer, kicker, intake));

    // B: spin up shooter then feed (low goal shot)
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whileTrue(
            new SequentialCommandGroup(
                new RunCommand(() -> {
                  shooter.setVelocity(ShooterConstants.kLowVelocity);
                  kicker.reverseKicker();
                }, shooter, kicker)
                    .withTimeout(1),
                new RunCommand(() -> {
                  shooter.setVelocity(ShooterConstants.kLowVelocity);
                  kicker.startKicker();
                  spindexer.startSpindexer();
                  intake.spinIntake();
                }, shooter, kicker, spindexer, intake))
                .finallyDo(() -> {
                  shooter.stopShooter();
                  kicker.stopKicker();
                  spindexer.stopSpindexer();
                  intake.stopIntake();
                }));

    // Left trigger: manual kicker forward
    new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5)
        .whileTrue(new RunCommand(
            () -> kicker.startKicker(), kicker)
            .finallyDo(
                () -> kicker.stopKicker()));

    // Left bumper: lock wheels in X formation
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(
            () -> drive.setX(),
            drive));

    // Back: toggle driver camera mode
    new JoystickButton(driverController, XboxController.Button.kBack.value)
        .onTrue(new InstantCommand(
            () -> vision.toggleDriverCam()));

    // Right bumper: toggle intake + spindexer + kicker sequence
    new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
        .onTrue(new InstantCommand(
            () -> {
              if (intake.isSpinning()) {
                intake.stopIntake();
                spindexer.stopSpindexer();
                kicker.stopKicker();
              } else {
                intake.spinIntake();
                spindexer.startSpindexer();
                kicker.reverseKicker();
              }
            }, intake, spindexer, kicker));

    // Right trigger: stationary aimbot
    new Trigger(() -> driverController.getRightTriggerAxis() > 0.5)
        .whileTrue(new StationaryAimbotCommand(drive, shooter, kicker, spindexer, intake, false, false));

    // D-pad up/down: adjust aimbot fudge factor
    new Trigger(() -> driverController.getPOV() == 0)
        .onTrue(new InstantCommand(() -> StationaryAimbotCommandData.addSixInches()));
    new Trigger(() -> driverController.getPOV() == 180)
        .onTrue(new InstantCommand(() -> StationaryAimbotCommandData.minusSixInches()));
  }

  /**
   * Returns the command to run during autonomous.
   *
   * @return The autonomous command.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Pose2d[] getAutoPoses() {
    try {
      return PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected().getName())
          .stream()
          .flatMap(path -> path.getPathPoses().stream())
          .toArray(Pose2d[]::new);
    } catch (IOException | ParseException e) {
      DriverStation.reportError("Failed to load auto poses: " + e.getMessage(), e.getStackTrace());
      return new Pose2d[0];
    }
  }

  public DriveSubsystem getDriveSubsystem() {
    return drive;
  }

  public Vision getVision() {
    return vision;
  }
}
