// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.aimbot.StationaryAimbotCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.IOException;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem drive = new DriveSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final SpindexerSubsystem spindexer = new SpindexerSubsystem();
  private final KickerSubsystem kicker = new KickerSubsystem();
  private final Vision vision = new Vision(drive);

  private final SlewRateLimiter xSlewRateLimiter;
  private final SlewRateLimiter ySlewRateLimiter;
  private final SlewRateLimiter rSlewRateLimiter;

  // The driver's controller
  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);

  // PathPlanner choose auto from SmartDashboard
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // PathPlanner commands
    NamedCommands.registerCommand("Start Intake", new InstantCommand(() -> intake.spinIntake()));
    NamedCommands.registerCommand("Stop Intake", new InstantCommand(() -> intake.stopIntake()));
    NamedCommands.registerCommand("Stationary Aimbot",
        new StationaryAimbotCommand(drive, shooter, kicker, spindexer));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the button bindings
    configureButtonBindings();

    // Create slew rate limiters to prevent stuttering
    xSlewRateLimiter = new SlewRateLimiter(1.8); // Rate limit is in seconds to max
    ySlewRateLimiter = new SlewRateLimiter(1.8);
    rSlewRateLimiter = new SlewRateLimiter(2.5);

    // Configure default commands
    drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> drive.drive(
                -MathUtil.applyDeadband(xSlewRateLimiter.calculate(driverController.getLeftY()),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(ySlewRateLimiter.calculate(driverController.getLeftX()),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(rSlewRateLimiter.calculate(driverController.getRightX()),
                    OIConstants.kDriveDeadband),
                true),
            drive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whileTrue(new StartEndCommand(
            () -> intake.reverseIntake(),
            () -> intake.stopIntake(),
            intake));

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

    // Have to actually turn this into a command probably, need spindexer + kicker
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whileTrue(
            new SequentialCommandGroup(
                new RunCommand(() -> shooter.setVelocity(ShooterSubsystem.lowVel), shooter)
                    .withTimeout(1), // spin up shooter alone first
                new RunCommand(() -> {
                  shooter.setVelocity(ShooterSubsystem.lowVel);
                  kicker.startKicker();
                  spindexer.startSpindexer();
                }, shooter, kicker, spindexer)).finallyDo(() -> {
                  shooter.stopShooter();
                  kicker.stopKicker();
                  spindexer.stopSpindexer();
                }));

    new JoystickButton(driverController, XboxController.Button.kX.value)
        .whileTrue(
            new SequentialCommandGroup(
                new RunCommand(() -> shooter.setVelocity(ShooterSubsystem.midVel), shooter)
                    .withTimeout(1), // spin up shooter alone first
                new RunCommand(() -> {
                  shooter.setVelocity(ShooterSubsystem.midVel);
                  kicker.startKicker();
                  spindexer.startSpindexer();
                }, shooter, kicker, spindexer)).finallyDo(() -> {
                  shooter.stopShooter();
                  kicker.stopKicker();
                  spindexer.stopSpindexer();
                }));

    /*
     * new JoystickButton(driverController, XboxController.Button.kStart.value)
     * .onTrue(new InstantCommand(
     * () -> drive.setX(),
     * drive));
     */

    new JoystickButton(driverController, XboxController.Button.kBack.value)
        .onTrue(new InstantCommand(
            () -> vision.toggleDriverCam()));

    new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
        .onTrue(new InstantCommand(
            () -> {
              if (intake.intakeIsSpinning) {
                intake.stopIntake();
                spindexer.stopSpindexer();
              } else {
                intake.spinIntake();
                spindexer.startSpindexer();
              }
            }, intake, spindexer));

    new Trigger(() -> driverController.getRightTriggerAxis() > 0.5)
        .whileTrue(new StationaryAimbotCommand(drive, shooter, kicker, spindexer));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
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
      e.printStackTrace();
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
