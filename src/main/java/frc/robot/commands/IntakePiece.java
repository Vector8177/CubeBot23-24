package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.GamePiece;
import frc.robot.subsystems.Intake;


public class IntakePiece extends CommandBase {

  
   
    private double time;
    private final Intake intakeSubsystem;
    private Timer timer;

    private GamePiece gamePiece;


    public IntakePiece(Intake intakeSubsystem, double time, GamePiece gamePiece) {
        this.time = time;
        this.gamePiece = gamePiece; 
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
        switch(gamePiece){
            case CONEINTAKE:
            intakeSubsystem.setMotor(-Constants.IntakeConstants.coneIntakeSpeed);
                break;
            case CONEOUTTAKE:
             intakeSubsystem.setMotor(Constants.IntakeConstants.coneIntakeSpeed);
                break;
            case CUBEINTAKE:
            intakeSubsystem.setMotor(Constants.IntakeConstants.cubeIntakeSpeed);
                break;
            case CUBEOUTTAKE:
            intakeSubsystem.setMotor(-Constants.IntakeConstants.cubeIntakeSpeed);
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
        if (gamePiece == GamePiece.CUBEOUTTAKE || gamePiece== GamePiece.CONEOUTTAKE) {
            return timer.get() > .5;
        }

        else if (gamePiece == GamePiece.CONEINTAKE && intakeSubsystem.getPDMCurrent() >= Constants.IntakeConstants.maxCurrentIntake
                && timer.hasElapsed(.3)) {
            return true;
        }
        //make two different things 

       else  if (gamePiece == GamePiece.CUBEINTAKE && intakeSubsystem.getPDMCurrent() >= Constants.IntakeConstants.maxCurrentIntake
                && timer.hasElapsed(.3)) {
            return true;
        }

        return false;
    }
}
