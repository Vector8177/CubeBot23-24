package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Intake;


public class IntakePiece extends CommandBase {

  
   
    private double time;
    private final Intake intakeSubsystem;
    private Timer timer;
    private boolean intaking;

    private Supplier<GamePiece> gamePiece;


    public IntakePiece(Intake intakeSubsystem, double time, Supplier<GamePiece> gamePiece, boolean intaking) {
        this.time = time;
        this.gamePiece = gamePiece; 
        this.intaking = intaking;
        this.intakeSubsystem = intakeSubsystem;
       

        this.timer = new Timer();

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        System.out.println("IntakeCmd started");

    }

    @Override
    public void execute() {
        switch(gamePiece.get()){
            case CONE:
                if(intaking){
            intakeSubsystem.setMotor(-Constants.IntakeConstants.intakeSpeed);
                }else{
             intakeSubsystem.setMotor(Constants.IntakeConstants.coneOuttakeSpeed);
                }
                break;
            case CUBE:
            if(intaking){
            intakeSubsystem.setMotor(Constants.IntakeConstants.intakeSpeed);
        }else{
            intakeSubsystem.setMotor(-Constants.IntakeConstants.cubeOuttakeSpeed);
        }
                break;
          
           

        }
      
      
      

    }

    @Override
    public void end(boolean interrupted) {

        timer.stop();
        timer.reset();

        intakeSubsystem.setMotor(0);
        System.out.println("IntakeCmd ended");
    }

    @Override
    public boolean isFinished() {
        if (!intaking) {
            return timer.get() > .5;
        }

        else if (intaking && intakeSubsystem.getPDMCurrent() >= Constants.IntakeConstants.maxCurrentIntake
                && timer.hasElapsed(.3)) {
            return true;
        }
        //make two different things 

       else  if (intaking && intakeSubsystem.getPDMCurrent() >= Constants.IntakeConstants.maxCurrentIntake
                && timer.hasElapsed(.3)) {
            return true;
        }

        return false;
    }
}
