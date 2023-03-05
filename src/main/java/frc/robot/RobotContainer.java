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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Position;
import frc.robot.Constants.SEGMENT;
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
  private static final int motorControl = XboxController.Axis.kLeftTrigger.value;
  /* Operator Controls */
  private static final int elevatorAxis = XboxController.Axis.kLeftY.value;
  private static final int wristAxis = XboxController.Axis.kRightY.value;
  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Intake s_Intake = new Intake();
  private final Elevator s_Elevator = new Elevator();
  private final Wrist s_Wrist = new Wrist();
  private final PhotonVisionWrapper s_PhotonVisionWrapper = s_Swerve.getCamera();

  /* Autonomous Mode Chooser */
  private final SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();

  /* Robot State */
  private static GamePiece gamePiece = GamePiece.CUBE;

  /* Autonomous Modes */
  PathPlannerTrajectory moveForward = PathPlanner.loadPath("Move Forward",
      Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  PathPlannerTrajectory sCurve = PathPlanner.loadPath("S Curve",
      Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  PathPlannerTrajectory sussy = PathPlanner.loadPath("sussy",
      Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  PathPlannerTrajectory autobalance = PathPlanner.loadPath("Autobalance", 
    Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared); 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

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

    // Configure the button bindings
    configureButtonBindings();

    // Configure Smart Dashboard options
    configureSmartDashboard();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   *  <p>
   * This method binds the buttons to commands. 
   * The x button is binded to AutoBalancing. 
   * Y button is for swerve
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    // driver.x().onTrue(new AutoBalancing(s_Swerve));
    driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    // driver.povDown().onTrue(new segmentLineUp(s_Swerve, Constants.SEGMENT.CUBE_3, () -> s_Swerve.getPoint()));
    // driver.povDown().onTrue(new segmentLineUp(s_Swerve, segmentLineUp.SEGMENT.CUBE_3, () -> s_Swerve.getPoint()));

    
  
    operator.povUp().onTrue(new IntakePiece(s_Intake, .3, GamePiece.CONE, true));
    operator.povDown().onTrue(new IntakePiece(s_Intake, .3, GamePiece.CONE, false));
    
    operator.povLeft().onTrue(new IntakePiece(s_Intake, .3, GamePiece.CUBE, true));
    operator.povRight().onTrue(new IntakePiece(s_Intake, .3, GamePiece.CUBE, false));
  

    operator.y().onTrue(new SetPosition(s_Wrist, s_Elevator, Position.HIGH, gamePiece));
    operator.b().onTrue(new SetPosition(s_Wrist, s_Elevator, Position.MID, gamePiece));
    operator.a().onTrue(new SetPosition(s_Wrist, s_Elevator, Position.LOW, gamePiece));
   

  

   

  }
  
  private void configureSmartDashboard() {
    autoChooser.setDefaultOption("Move forward", moveForward);
    autoChooser.addOption("S curve", sCurve);
    autoChooser.addOption("SUSSY - CADEN", sussy);
    autoChooser.addOption("Autobalance :)", autobalance); 

    SmartDashboard.putData(autoChooser);
  }

  /**
   * TODOs
   */
  public void disabledInit() {
    s_Swerve.resetToAbsolute();
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

  public void setGamePiece(GamePiece piece){
    gamePiece = piece;
  }

  public GamePiece getGamePiece(){
    return gamePiece;
  }
}