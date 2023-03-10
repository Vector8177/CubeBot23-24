// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Position;
import frc.robot.Constants.Intake.EjectSpeed;
import frc.robot.Constants.LEDs.LEDMode;
import frc.robot.autos.AutoBalancing;
import frc.robot.Constants.GamePiece;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDs.LEDs;

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
        private final LEDs s_LEDs = new LEDs();

        public static GamePiece gamePiece = GamePiece.CONE;

        /* Autonomous Mode Chooser */
        private final SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();

        /* Autonomous */
        private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                        s_Swerve::getPose,
                        s_Swerve::resetOdometry,
                        Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                        new PIDConstants(Constants.Autonomous.kPXController, 0, 0),
                        new PIDConstants(Constants.Autonomous.kPThetaController,
                                        0,
                                        0),
                        s_Swerve::setModuleStates,
                        eventMap,
                        true,
                        s_Swerve);

        private static Map<String, Command> eventMap = new HashMap<>();
        {

                eventMap.put("setStandbyPosition",
                                new SetPosition(s_Wrist, s_Elevator, Position.STANDBY, () -> gamePiece));
                eventMap.put("setCone3Position",
                                new SequentialCommandGroup(
                                                s_Elevator.setPose(Position.CONEHIGH.getElev()),
                                                new WaitCommand(.5),
                                                s_Wrist.setPose(Position.CONEHIGH.getWrist()+.05),
                                                new WaitCommand(1)));
                eventMap.put("setCube3Position",
                        new SequentialCommandGroup(
                                s_Wrist.setPose(Position.CUBEHIGH.getWrist()),
                                s_Elevator.setPose(Position.CUBEHIGH.getElev()),
                                new WaitCommand(1)));

                eventMap.put("setCubeIntakePosition",
                                new SetPosition(s_Wrist, s_Elevator, Position.CUBEINTAKE, () -> GamePiece.CUBE));
                eventMap.put("setStandingConeIntakePosition", new SetPosition(s_Wrist, s_Elevator,
                                Position.STANDINGCONEINTAKE, () -> GamePiece.CONE));

                eventMap.put("setTippedConeIntakePosition", new SetPosition(s_Wrist, s_Elevator,
                                Position.TIPPEDCONEINTAKE, () -> GamePiece.CONE));

                eventMap.put("coneDeposit", new OuttakePiece(s_Intake, .3, () -> GamePiece.CONE, EjectSpeed.NORMAL));
                eventMap.put("cubeDeposit", new OuttakePiece(s_Intake, .3, () -> GamePiece.CUBE, EjectSpeed.NORMAL));

                eventMap.put("runCubeIntake3", new OuttakePiece(s_Intake, 3, () -> GamePiece.CONE, EjectSpeed.NORMAL));
                eventMap.put("runConeIntake3", new OuttakePiece(s_Intake, 3, () -> GamePiece.CUBE, EjectSpeed.NORMAL));

                eventMap.put("wait1Seconds", new WaitCommand(1));

                eventMap.put("AutoBalance", new AutoBalancing(s_Swerve, true));
        }

        private final PathPlannerTrajectory moveForward = PathPlanner.loadPath("Move Forward",
                        Constants.Autonomous.kMaxSpeedMetersPerSecond,
                        Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared);

        private final PathPlannerTrajectory sCurve = PathPlanner.loadPath("S Curve",
                        Constants.Autonomous.kMaxSpeedMetersPerSecond,
                        Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared);

        private final PathPlannerTrajectory autoBalance = PathPlanner.loadPath("Autobalance",
                        1,
                        3);

        private final PathPlannerTrajectory backnForth = PathPlanner.loadPath("BacknForth",
                        Constants.Autonomous.kMaxSpeedMetersPerSecond,
                        Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared);

        private final PathPlannerTrajectory coneCubeDeposit = PathPlanner.loadPath("coneCubeDeposit",
                        3,
                        1);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                CameraServer.startAutomaticCapture();

                // Sets each subsystem's default commands
                setDefaultCommands();

                // Configure the button bindings
                configureButtonBindings();

                // Configure Smart Dashboard options
                configureSmartDashboard();

                // Configure autonomous routines
                configureAutonomousPaths();
        }

        private void setDefaultCommands() {
                s_Swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                s_Swerve,
                                                () -> -driver.getRawAxis(translationAxis),
                                                () -> -driver.getRawAxis(strafeAxis),
                                                () -> -driver.getRawAxis(rotationAxis),
                                                () -> driver.povDown().getAsBoolean(),
                                                () -> driver.leftBumper().getAsBoolean(),
                                                () -> driver.rightBumper().getAsBoolean()));

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

        private void configureAutonomousPaths() {

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
         */
        private void configureButtonBindings() {
                /* Driver Buttons */
                driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
                driver.x().onTrue(new AutoBalancing(s_Swerve, true));
                driver.rightTrigger().onTrue(new OuttakePiece(s_Intake, .5, () -> getGamePiece(), EjectSpeed.NORMAL));

                /* Operator Buttons */
                operator.povUp().onTrue(
                                new SequentialCommandGroup(new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
                                                new SetPosition(s_Wrist, s_Elevator, Position.STANDINGCONEINTAKE,
                                                                () -> GamePiece.CONE)));

                operator.povLeft().onTrue(new SequentialCommandGroup(
                                new InstantCommand(() -> setGamePiece(GamePiece.CUBE)),
                                new SetPosition(s_Wrist, s_Elevator, Position.CUBEINTAKE, () -> GamePiece.CUBE)));

                operator.povDown().onTrue(new SequentialCommandGroup(
                                new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
                                new SetPosition(s_Wrist, s_Elevator, Position.TIPPEDCONEINTAKE, () -> GamePiece.CONE)));

                operator.povRight()
                                .onTrue(new SequentialCommandGroup(
                                                new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
                                                new SetPosition(s_Wrist, s_Elevator, Position.HUMANPLAYERINTAKE,
                                                                () -> GamePiece.CONE)));

                operator.leftBumper()
                                .onTrue(new SetPosition(s_Wrist, s_Elevator, Position.STANDBY, () -> getGamePiece()));

                operator.leftTrigger().onTrue(new OuttakePiece(s_Intake, .5, () -> getGamePiece(), EjectSpeed.NORMAL));
                operator.x().onTrue(new OuttakePiece(s_Intake, .5, () -> getGamePiece(), EjectSpeed.FAST));

                operator.rightBumper().onTrue(new InstantCommand(() -> s_LEDs.toggleHPSignal()));

                operator.y().onTrue(new SetPosition(s_Wrist, s_Elevator, Position.HIGH, () -> getGamePiece()));
                operator.b().onTrue(new SetPosition(s_Wrist, s_Elevator, Position.MID, () -> getGamePiece()));
                operator.a().onTrue(new SetPosition(s_Wrist, s_Elevator, Position.LOW, () -> getGamePiece()));

        }

        private void configureSmartDashboard() {
                // Autonomous Mode Chooser
                autoChooser.setDefaultOption("Move forward", moveForward);
                autoChooser.addOption("S curve", sCurve);
                autoChooser.addOption("Auto balance", autoBalance);
                autoChooser.addOption("Cone and Cube", coneCubeDeposit);
                autoChooser.addOption("Back and Forth", backnForth);
                SmartDashboard.putData(autoChooser);
        }

        /**
         * Ran once the robot is put in disabled
         */
        public void disabledInit() {
                s_Swerve.resetToAbsolute();
                s_LEDs.setLEDMode(LEDMode.VECTORWAVE);
        }

        public static GamePiece getGamePiece() {
                return gamePiece;
        }

        public static void setGamePiece(GamePiece piece) {
                gamePiece = piece;
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

        public Command getAutonomousCommand() {
                // Executes the autonomous command chosen in smart dashboard
                s_Swerve.getField().getObject("Field").setTrajectory(autoChooser.getSelected());
                return autoBuilder.fullAuto(autoChooser.getSelected());
        }
}
