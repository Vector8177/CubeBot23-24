package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.EjectSpeed;
import frc.robot.subsystems.Intake;

public class OuttakePiece extends CommandBase {

    private double time;
    private final Intake intakeSubsystem;
    private Timer timer;
    private EjectSpeed eject;
    private Supplier<GamePiece> gamePiece;

    public OuttakePiece(Intake intakeSubsystem, double time, Supplier<GamePiece> gamePiece, EjectSpeed eject) {
        this.time = time;
        this.gamePiece = gamePiece;
        this.eject = eject;
        this.intakeSubsystem = intakeSubsystem;

        this.timer = new Timer();

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        switch (gamePiece.get()) {
            case CONE:
                intakeSubsystem.setMotor((eject == EjectSpeed.NORMAL) ? Constants.Intake.coneOuttakeSpeed : Constants.Intake.coneShootSpeed);
                break;
            case CUBE:
                intakeSubsystem.setMotor(-Constants.Intake.cubeOuttakeSpeed);
                break;

        }

    }

    @Override
    public void end(boolean interrupted) {

        timer.stop();
        timer.reset();

        intakeSubsystem.setMotor(0);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > time;
    }
}
