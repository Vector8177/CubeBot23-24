package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeCmd extends CommandBase {
    
    private boolean intake; 
    private boolean cone; 
    private boolean wrist; 
    private double time; 
    private final Intake intakeSubsystem; 
    private Timer timer;
   
    
    
    public IntakeCmd(Intake intakeSubsystem, double time, boolean intake, boolean cone, boolean wrist){
        this.time = time; 
        this.intake = intake; 
        this.cone = cone; 
        this.wrist = wrist;  
        this.intakeSubsystem = intakeSubsystem; 
        
        this.timer = new Timer();
        
        
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        
        System.out.println("IntakeCmd started"); 
        
    }
    @Override
    public void execute(){
        if(cone && intake){
            intakeSubsystem.setMotor(-Constants.IntakeConstants.coneIntakeSpeed);
        }
        if(!cone && intake){
            intakeSubsystem.setMotor(Constants.IntakeConstants.cubeIntakeSpeed);
            
        }
        if(cone && !intake){
            intakeSubsystem.setMotor(-Constants.IntakeConstants.coneIntakeSpeed);
        }
        if(!cone && !intake){
            intakeSubsystem.setMotor(Constants.IntakeConstants.cubeIntakeSpeed);
            
        }
        
    }
   
    @Override
    public void end(boolean interrupted){
        
            timer.stop();
            timer.reset();
        
        intakeSubsystem.setMotor(0);
        System.out.println("IntakeCmd ended"); 
    }
    
    @Override
    public boolean isFinished(){
        if(cone && intakeSubsystem.getPDMCurrent() >= Constants.IntakeConstants.maxCurrentIntake && timer.hasElapsed(.3)){
            return true;
        }
        if(!cone && intakeSubsystem.getPDMCurrent() >= Constants.IntakeConstants.maxCurrentIntake  && timer.hasElapsed(.3) ){
            return true; 
        }
        if(!intake){
            return timer.hasElapsed(.5); 
        }
        return false; 
    }
}

