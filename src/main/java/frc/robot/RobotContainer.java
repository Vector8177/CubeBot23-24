// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Position;
import frc.robot.Constants.Intake.EjectSpeed;
import frc.robot.Constants.LEDs.LEDMode;
import frc.robot.Constants.Wrist.PIDFFmode;
import frc.robot.autos.AutoBalancing;
import frc.robot.Constants.GamePiece;
import frc.robot.commands.*;
import frc.robot.commands.TimedIntake.Direction;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOSparkMax;
// import frc.robot.Constants.SEGMENT;
// import frc.robot.autos.segmentLineUp;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOPhoton;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSparkMax;

public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    /* Drive Controls */
    private static final int translationAxis = XboxController.Axis.kLeftY.value;
    private static final int strafeAxis = XboxController.Axis.kLeftX.value;
    private static final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Operator Controls */
    private static final int elevatorAxis = XboxController.Axis.kLeftY.value;
    private static final int wristAxis = XboxController.Axis.kRightY.value;
    private static final int intakeTrigger = XboxController.Axis.kRightTrigger.value;

    /* Subsystems */
    private final Vision s_Vision;
    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final Elevator s_Elevator;
    private final Wrist s_Wrist;
    private final LEDs s_LEDs = new LEDs();

    public static GamePiece gamePiece = GamePiece.CONE;

    /* Autonomous Mode Chooser */
    private final LoggedDashboardChooser<PathPlannerTrajectory> autoChooser = new LoggedDashboardChooser<>(
            "Auto Choices");

    // private final SendableChooser<PathPlannerTrajectory> autoChooser = new
    // SendableChooser<>();

    /* Autonomous */
    private final SwerveAutoBuilder autoBuilder;

    private static Map<String, Command> eventMap;
    private final PathPlannerTrajectory autoBalance = PathPlanner.loadPath("Autobalance",
            1,
            3);

    private final PathPlannerTrajectory coneCubeBalance = PathPlanner.loadPath("coneCubeBalance",
            3.5,
            1.5);

    private final PathPlannerTrajectory coneCubeDeposit = PathPlanner.loadPath("coneCubeDeposit",
            3,
            1);

    private final PathPlannerTrajectory csConeCubeBalance = PathPlanner.loadPath("csConeCubeBalance",
            2.0,
            1.0);

    private final PathPlannerTrajectory threePieceAuto = PathPlanner.loadPath("threePieceAuto",
            3.5,
            2.5);

    private final PathPlannerTrajectory twoPlusBalance = PathPlanner.loadPath("score2Balance",
            3.5,
            2.5);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.getMode()) {
            // Real robot, instantiate hardware IO implementations
            case REAL:
                s_Vision = new Vision(
                        new CameraIOPhoton(Constants.Vision.leftCameraName),
                        new CameraIOPhoton(Constants.Vision.rightCameraName));
                s_Swerve = new Swerve(new GyroIOPigeon2(),
                        new ModuleIOSparkMax(Constants.Swerve.Mod0.constants),
                        new ModuleIOSparkMax(Constants.Swerve.Mod1.constants),
                        new ModuleIOSparkMax(Constants.Swerve.Mod2.constants),
                        new ModuleIOSparkMax(Constants.Swerve.Mod3.constants),
                        s_Vision);
                s_Intake = new Intake(new IntakeIOSparkMax());
                s_Wrist = new Wrist(new WristIOSparkMax());
                s_Elevator = new Elevator(new ElevatorIOSparkMax());
                break;

            // Replayed robot, disable IO implementations
            default:
                s_Vision = new Vision(
                        new CameraIO() {
                        },
                        new CameraIO() {
                        });
                s_Swerve = new Swerve(new GyroIO() {
                },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        s_Vision);
                s_Intake = new Intake(new IntakeIO() {
                });
                s_Wrist = new Wrist(new WristIO() {
                });
                s_Elevator = new Elevator(new ElevatorIO() {
                });
                break;

        }

        CameraServer.startAutomaticCapture();

        // Sets each subsystem's default commands
        setDefaultCommands();

        // Configure the button bindings
        configureButtonBindings();

        // Configure Smart Dashboard options
        configureSmartDashboard();

        // Configure autonomous events
        configureAutonomousEvents();

        autoBuilder = new SwerveAutoBuilder(
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
    }

    private void setDefaultCommands() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        s_LEDs,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> driver.povDown().getAsBoolean(),
                        () -> driver.leftBumper().getAsBoolean(),
                        () -> driver.rightBumper().getAsBoolean(),
                        () -> driver.a().getAsBoolean()));

        s_Elevator.setDefaultCommand(
                new TeleopElevator(
                        s_Elevator,
                        () -> operator.getRawAxis(elevatorAxis)));

        s_Wrist.setDefaultCommand(
                new TeleopWrist(
                        s_Wrist,
                        () -> operator.getRawAxis(wristAxis)));

        s_Intake.setDefaultCommand(
                new TeleopIntake(s_Intake, s_Wrist,
                        () -> operator.getRawAxis(intakeTrigger)));
    }

    private void configureAutonomousEvents() {
        eventMap = new HashMap<>();
        eventMap.put("setStandbyPosition",
                new SetPosition(s_Wrist, s_Elevator, Position.STANDBY, () -> gamePiece));

        eventMap.put("setCone3Position",
                new SequentialCommandGroup(
                        new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
                        s_Elevator.moveElevator(Position.CONEHIGH.getElev()),
                        new WaitCommand(0.5),
                        s_Wrist.moveWrist(Position.CONEHIGH.getWrist())
                                .raceWith(new WaitCommand(1))));

        eventMap.put("setCube3Position",
                new SequentialCommandGroup(
                        new InstantCommand(() -> setGamePiece(GamePiece.CUBE)),
                        s_Wrist.moveWrist(Position.CUBEHIGH.getWrist()),
                        s_Elevator.moveElevator(Position.CUBEHIGH.getElev())));

        eventMap.put("setCone2Position",
                new SequentialCommandGroup(
                        new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
                        s_Elevator.moveElevator(Position.CONEMID.getElev()),
                        s_Wrist.moveWrist(Position.CONEMID.getWrist())
                                .raceWith(new WaitCommand(1.5))));

        eventMap.put("setCube2Position",
                new SequentialCommandGroup(
                        new InstantCommand(() -> setGamePiece(GamePiece.CUBE)),
                        s_Wrist.moveWrist(Position.CUBEMID.getWrist()),
                        s_Elevator.moveElevator(Position.CUBEMID.getElev())));

        eventMap.put("setCubeIntakePosition",
                new SequentialCommandGroup(
                        new InstantCommand(() -> setGamePiece(GamePiece.CUBE)),
                        new SetPosition(s_Wrist, s_Elevator, Position.CUBEINTAKE,
                                () -> GamePiece.CUBE)));

        eventMap.put("setStandingConeIntakePosition",
                new SequentialCommandGroup(
                        new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
                        new SetPosition(s_Wrist, s_Elevator, Position.STANDINGCONEINTAKE,
                                () -> GamePiece.CONE)));

        eventMap.put("setTippedConeIntakePosition",
                new SequentialCommandGroup(
                        new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
                        new SetPosition(s_Wrist, s_Elevator, Position.TIPPEDCONEINTAKE,
                                () -> GamePiece.CONE)));

        eventMap.put("coneDeposit",
                new SequentialCommandGroup(
                        new InstantCommand(() -> s_Wrist.setPIDFFMode(PIDFFmode.UNWEIGHTED)),
                        new TimedIntake(s_Intake, .3, GamePiece.CONE, EjectSpeed.CONENORMAL,
                                Direction.OUTTAKE)));

        eventMap.put("cone3Deposit",
                new SequentialCommandGroup(
                        new InstantCommand(() -> s_Wrist
                                .setPIDFFMode(PIDFFmode.UNWEIGHTED)),
                        new TimedIntake(s_Intake, .1,
                                GamePiece.CONE,
                                EjectSpeed.CONEFAST,
                                Direction.OUTTAKE),
                        new ParallelCommandGroup(
                                new TimedIntake(s_Intake,
                                        .2,
                                        GamePiece.CONE,
                                        EjectSpeed.CONESLOW,
                                        Direction.OUTTAKE),
                                new SetPosition(s_Wrist,
                                        s_Elevator,
                                        Position.CONEHIGHUP,
                                        () -> GamePiece.CUBE))));

        eventMap.put("cubeDeposit",
                new TimedIntake(s_Intake, .3, GamePiece.CUBE, EjectSpeed.CUBENORMAL,
                        Direction.OUTTAKE));

        eventMap.put("runCubeIntake3",
                new TimedIntake(s_Intake, 3, GamePiece.CUBE, EjectSpeed.CUBENORMAL, Direction.INTAKE));

        eventMap.put("runConeIntake3",
                new SequentialCommandGroup(
                        new InstantCommand(() -> s_Wrist.setPIDFFMode(PIDFFmode.WEIGHTED)),
                        new TimedIntake(s_Intake, 3, GamePiece.CONE, EjectSpeed.CONEINTAKE,
                                Direction.INTAKE)));

        eventMap.put("wait1Seconds", new WaitCommand(1));

        eventMap.put("AutoBalance", new AutoBalancing(s_Swerve, true));
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
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.setYaw(Rotation2d.fromDegrees(0))));
        driver.x().onTrue(new SelectCommand(
                Map.ofEntries(
                        Map.entry(GamePiece.CUBE,
                                new TimedIntake(s_Intake, .5, GamePiece.CUBE,
                                        EjectSpeed.CUBENORMAL,
                                        Direction.OUTTAKE)),
                        Map.entry(GamePiece.CONE,
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> s_Wrist
                                                .setPIDFFMode(PIDFFmode.UNWEIGHTED)),
                                        new TimedIntake(s_Intake, .1,
                                                GamePiece.CONE,
                                                EjectSpeed.CONEFAST,
                                                Direction.OUTTAKE),
                                        new ParallelCommandGroup(
                                                new TimedIntake(s_Intake,
                                                        .2,
                                                        GamePiece.CONE,
                                                        EjectSpeed.CONESLOW,
                                                        Direction.OUTTAKE),
                                                new SetPosition(s_Wrist,
                                                        s_Elevator,
                                                        Position.CONEHIGHUP,
                                                        () -> GamePiece.CUBE))))),
                () -> gamePiece));
        /*
         * driver.b().whileTrue(
         * autoBuilder.followPath(
         * segmentLineUp.getTrajectory(SEGMENT.HUMANPLAYER,
         * () -> s_Swerve.getPose())));
         */
        driver.leftTrigger().onTrue(new SelectCommand(
                Map.ofEntries(
                        Map.entry(GamePiece.CUBE,
                                new TimedIntake(s_Intake, .5, GamePiece.CUBE,
                                        EjectSpeed.CUBEFAST,
                                        Direction.OUTTAKE)),
                        Map.entry(GamePiece.CONE,
                                new TimedIntake(s_Intake, .5, GamePiece.CONE,
                                        EjectSpeed.CONEFAST,
                                        Direction.OUTTAKE))),
                () -> gamePiece));
        driver.rightTrigger().onTrue(new SelectCommand(
                Map.ofEntries(
                        Map.entry(GamePiece.CUBE,
                                new TimedIntake(s_Intake, .5, GamePiece.CUBE,
                                        EjectSpeed.CUBENORMAL,
                                        Direction.OUTTAKE)),
                        Map.entry(GamePiece.CONE,
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> s_Wrist
                                                .setPIDFFMode(PIDFFmode.UNWEIGHTED)),
                                        new TimedIntake(s_Intake, .5,
                                                GamePiece.CONE,
                                                EjectSpeed.CONENORMAL,
                                                Direction.OUTTAKE)))),
                () -> gamePiece));

        /* Operator Buttons */
        operator.povUp().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
                new SetPosition(s_Wrist, s_Elevator, Position.STANDINGCONEINTAKE,
                        () -> GamePiece.CONE)));

        operator.povLeft().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> setGamePiece(GamePiece.CUBE)),
                new SetPosition(s_Wrist, s_Elevator, Position.CUBEINTAKE, () -> GamePiece.CUBE)));

        operator.povDown().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
                new SetPosition(s_Wrist, s_Elevator, Position.TIPPEDCONEINTAKE, () -> GamePiece.CONE)));

        operator.povRight().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
                new SetPosition(s_Wrist, s_Elevator, Position.HUMANPLAYERINTAKE,
                        () -> GamePiece.CONE)));

        operator.leftBumper()
                .onTrue(new SetPosition(s_Wrist, s_Elevator, Position.STANDBY, () -> getGamePiece()));

        operator.leftTrigger().onTrue(new SelectCommand(
                Map.ofEntries(
                        Map.entry(GamePiece.CUBE,
                                new TimedIntake(s_Intake, .5, GamePiece.CUBE,
                                        EjectSpeed.CUBENORMAL,
                                        Direction.OUTTAKE)),
                        Map.entry(GamePiece.CONE,
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> s_Wrist
                                                .setPIDFFMode(PIDFFmode.UNWEIGHTED)),
                                        new TimedIntake(s_Intake, .5,
                                                GamePiece.CONE,
                                                EjectSpeed.CONENORMAL,
                                                Direction.OUTTAKE)))),
                () -> gamePiece));

        operator.x().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> setGamePiece(GamePiece.CONE)),
                new SetPosition(s_Wrist, s_Elevator, Position.DOUBSUBSTATIONINTAKE,
                        () -> GamePiece.CONE)));
        /*
         * operator.x().onTrue(new SelectCommand(
         * Map.ofEntries(
         * Map.entry(GamePiece.CUBE,
         * new TimedIntake(s_Intake, .5, GamePiece.CUBE,
         * EjectSpeed.CUBEFAST, Direction.OUTTAKE)),
         * Map.entry(GamePiece.CONE,
         * new TimedIntake(s_Intake, .5, GamePiece.CONE,
         * EjectSpeed.CONEFAST, Direction.OUTTAKE))),
         * () -> gamePiece));
         */
        operator.rightBumper().onTrue(new InstantCommand(() -> s_LEDs.toggleHPSignal()));

        operator.y().onTrue(new SelectCommand(
                Map.ofEntries(
                        Map.entry(GamePiece.CUBE,
                                new SetPosition(s_Wrist, s_Elevator, Position.HIGH,
                                        () -> getGamePiece())),
                        Map.entry(GamePiece.CONE, new SequentialCommandGroup(
                                s_Wrist.moveWrist(Position.STANDBY.getWrist()),
                                s_Elevator.moveElevator(Position.CONEHIGH.getElev()),
                                s_Wrist.moveWrist(Position.CONEHIGH.getWrist())))),
                () -> gamePiece));

        operator.b().onTrue(new SetPosition(s_Wrist, s_Elevator, Position.MID, () -> getGamePiece()));
        operator.a().onTrue(new SetPosition(s_Wrist, s_Elevator, Position.LOW, () -> getGamePiece()));

    }

    private void configureSmartDashboard() {
        // Autonomous Mode Chooser
        autoChooser.addDefaultOption("Cone PCube Balance", coneCubeBalance);
        autoChooser.addOption("Auto Balance", autoBalance);
        autoChooser.addOption("L3 Cone+Cube", coneCubeDeposit);
        autoChooser.addOption("Charge Station Cone PCube Balance", csConeCubeBalance);
        autoChooser.addOption("3 Game Piece Auto", threePieceAuto);
        autoChooser.addOption("Score 2 Plus Balance", twoPlusBalance);
        // autoChooser.addOption("Back and Forth", backnForth);
        // autoChooser.addOption("S curve", sCurve);

        // SmartDashboard.putData(autoChooser);
    }

    /**
     * Ran once the robot is put in disabled
     */
    public void disabledPeriodic() {
        s_Vision.updatePoseAlliance();
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
        s_Swerve.setYaw(autoChooser.get().getInitialPose().getRotation());
        Logger.getInstance().recordOutput("Trajectory", autoChooser.get());

        // Executes the autonomous command chosen in smart dashboard
        return new ParallelCommandGroup(
                new InstantCommand(
                        () -> s_Swerve.getField().getObject("Field").setTrajectory(
                                autoChooser.get())),
                autoBuilder.fullAuto(autoChooser.get()));
    }
}
