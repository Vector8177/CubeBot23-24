// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Position;
import frc.robot.Constants.GamePiece;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  /* Controllers */
  public final CommandXboxController driver = new CommandXboxController(0);
  public final CommandXboxController operator = new CommandXboxController(1);

  /* Drive Controls */
  private static final int translationAxis = XboxController.Axis.kLeftY.value;
  private static final int strafeAxis = XboxController.Axis.kLeftX.value;
  private static final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Operator Controls */
  private static final int elevatorAxis = XboxController.Axis.kLeftY.value;
  private static final int wristAxis = XboxController.Axis.kRightY.value;
  private static final int intakeTrigger = XboxController.Axis.kRightTrigger.value;

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Intake s_Intake = new Intake();
  private final Elevator s_Elevator = new Elevator();
  private final Wrist s_Wrist = new Wrist();

  /* Autonomous Mode Chooser */
  private final SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();
  private final SendableChooser<GamePiece> gamePieceChooser = new SendableChooser<>();

  /* Autonomous Modes */
  PathPlannerTrajectory moveForward = PathPlanner.loadPath("Move Forward",
      Constants.Autonomous.kMaxSpeedMetersPerSecond, Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared);
  PathPlannerTrajectory sCurve = PathPlanner.loadPath("S Curve",
      Constants.Autonomous.kMaxSpeedMetersPerSecond, Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared);
  PathPlannerTrajectory sussy = PathPlanner.loadPath("sussy",
      Constants.Autonomous.kMaxSpeedMetersPerSecond, Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared);
  PathPlannerTrajectory autobalance = PathPlanner.loadPath("Autobalance",
      Constants.Autonomous.kMaxSpeedMetersPerSecond, Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Sets the default command for each subsystem
    setDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

    // Configure Smart Dashboard options
    configureSmartDashboard();
  }

  private void setDefaultCommands() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> driver.povDown().getAsBoolean()));

    s_Elevator.setDefaultCommand(
        new TeleopElevator(
            s_Elevator,
            () -> operator.getRawAxis(elevatorAxis)));

    s_Wrist.setDefaultCommand(
        new TeleopWrist(
            s_Wrist,
            () -> operator.getRawAxis(wristAxis)));

    s_Intake.setDefaultCommand(
        new TeleopIntake(s_Intake,
            () -> operator.getRawAxis(intakeTrigger)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   * <p>
   * This method binds the buttons to commands.
   * The x button is binded to AutoBalancing.
   * Y button is for swerve
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    /* Operator Buttons */
    operator.povUp().onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> Intake.setGamePiece(GamePiece.CONE)),
            new SetPosition(s_Wrist, s_Elevator, Position.STANDINGCONEINTAKE, () -> Intake.getGamePiece())));

    operator.povLeft().onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> Intake.setGamePiece(GamePiece.CUBE)),
            new SetPosition(s_Wrist, s_Elevator, Position.CUBEINTAKE, () -> Intake.getGamePiece())));

    operator.povDown().onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> Intake.setGamePiece(GamePiece.CONE)),
            new SetPosition(s_Wrist, s_Elevator, Position.TIPPEDCONEINTAKE, () -> Intake.getGamePiece())));

    operator.povRight().onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> Intake.setGamePiece(GamePiece.CONE)),
            new SetPosition(s_Wrist, s_Elevator, Position.HUMANPLAYERINTAKE, () -> Intake.getGamePiece())));

    operator.leftBumper().onTrue(new SetPosition(s_Wrist, s_Elevator, Position.STANDBY, () -> Intake.getGamePiece()));

    operator.leftTrigger().onTrue(new OuttakePiece(s_Intake, .5, () -> Intake.getGamePiece()));

    operator.y().onTrue(new SetPosition(s_Wrist, s_Elevator, Position.HIGH, () -> Intake.getGamePiece()));
    operator.b().onTrue(new SetPosition(s_Wrist, s_Elevator, Position.MID, () -> Intake.getGamePiece()));
    operator.a().onTrue(new SetPosition(s_Wrist, s_Elevator, Position.LOW, () -> Intake.getGamePiece()));

  }

  private void configureSmartDashboard() {
    // Autonomous Mode Chooser
    autoChooser.setDefaultOption("Move forward", moveForward);
    autoChooser.addOption("S curve", sCurve);
    autoChooser.addOption("SUSSY - CADEN", sussy);
    autoChooser.addOption("Autobalance :)", autobalance);
    SmartDashboard.putData(autoChooser);

    // Game Piece Chooser
    gamePieceChooser.setDefaultOption("Cone", GamePiece.CONE);
    gamePieceChooser.addOption("Cube", GamePiece.CUBE);
    SmartDashboard.putData(gamePieceChooser);
  }

  /**
   * Ran once the robot is put in disabled
   */
  public void disabledInit() {
    s_Swerve.resetToAbsolute();
  }

  /**
   * Ran once the robot is put in teleoperated mode
   */
  public GamePiece getSelectedGamePiece() {
    return gamePieceChooser.getSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Executes the autonomous command chosen in smart dashboard
    return new executeTrajectory(s_Swerve, autoChooser.getSelected(), true);
  }
}