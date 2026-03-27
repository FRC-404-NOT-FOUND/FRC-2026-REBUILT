// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.aimbot.StationaryAimbotCommandData;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Vision vision;
  private final RobotContainer m_robotContainer;

  private boolean hasAutoRun = false;

  private final Field2d field = new Field2d();

  private final StructArrayPublisher<Pose2d> actualPathPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Actual auto path", Pose2d.struct).publish();

  private final StructArrayPublisher<Pose2d> desiredPathPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Desired auto path", Pose2d.struct).publish();

  private List<Pose2d> actualPath = new ArrayList<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    vision = m_robotContainer.getVision();
  }

  @Override
  public void robotInit() {
    StationaryAimbotCommandData.resetOffset();

    SmartDashboard.putData("Field", field);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    vision.periodic();

    field.setRobotPose(m_robotContainer.getDriveSubsystem().getPose());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    flipGyro();
    actualPath.clear();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
      desiredPathPublisher.set(m_robotContainer.getAutoPoses());
    }
    hasAutoRun = true;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    actualPath.add(m_robotContainer.getDriveSubsystem().getPose());
  }

  @Override
  public void autonomousExit() {
    actualPathPublisher.set(actualPath.toArray(new Pose2d[0]));
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if (!hasAutoRun) {
      flipGyro();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  private void flipGyro() {
    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
    m_robotContainer.getDriveSubsystem().zeroHeading();
    m_robotContainer.getDriveSubsystem().resetPose(
        new Pose2d(
            m_robotContainer.getDriveSubsystem().getPose().getTranslation(),
            isRed ? Rotation2d.fromDegrees(180) : new Rotation2d()));
  }
}