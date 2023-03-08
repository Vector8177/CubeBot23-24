package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Intake.EjectSpeed;
import frc.robot.subsystems.Intake;

public class OuttakePiece extends CommandBase {

    private double time;
    private final Intake s_Intake;
    private Timer timer;
    private EjectSpeed eject;
    private Supplier<GamePiece> gamePiece;

    public OuttakePiece(Intake s_Intake, double time, EjectSpeed eject) {
        this.time = time;
        this.eject = eject;
        this.s_Intake = s_Intake;
        this.timer = new Timer();

        addRequirements(s_Intake);
    }

    public OuttakePiece(Intake s_Intake, double time, Supplier<GamePiece> gamePiece, EjectSpeed eject) {
        this.time = time;
        this.eject = eject;
        this.s_Intake = s_Intake;
        this.gamePiece =  gamePiece;
        this.timer = new Timer();

        addRequirements(s_Intake);
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
                s_Intake.setMotor((eject == EjectSpeed.NORMAL) ? Constants.Intake.coneOuttakeSpeed
                        : Constants.Intake.coneShootSpeed);
                break;
            case CUBE:
                s_Intake.setMotor(-Constants.Intake.cubeOuttakeSpeed);
                break;
            
        }

    }

    @Override
    public void end(boolean interrupted) {

        timer.stop();
        timer.reset();

        s_Intake.setMotor(0);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > time;
    }
}
