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
    private final Intake s_Intake; 
    private Timer timer;
   
    
    
    public IntakeCmd(Intake s_Intake, double time, boolean intake, boolean cone, boolean wrist){
        this.time = time; 
        this.intake = intake; 
        this.cone = cone; 
        this.wrist = wrist;  
        this.s_Intake = s_Intake; 
        
        this.timer = new Timer();
        
        
        addRequirements(s_Intake);
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
            s_Intake.setMotor(-Constants.IntakeConstants.coneIntakeSpeed);
        }
        if(!cone && intake){
            s_Intake.setMotor(Constants.IntakeConstants.cubeIntakeSpeed);
        }
        if(cone && !intake){
            s_Intake.setMotor(Constants.IntakeConstants.coneIntakeSpeed);
        }
        if(!cone && !intake){
            s_Intake.setMotor(-Constants.IntakeConstants.cubeIntakeSpeed);
            
        }
        
    }
   
    @Override
    public void end(boolean interrupted){
        timer.stop();
        timer.reset();
        
        s_Intake.setMotor(0);
        System.out.println("IntakeCmd ended"); 
    }
    
    @Override
    public boolean isFinished(){
        if(!intake){
           
            return timer.get() >.5;
        }

        if(cone && s_Intake.getPDMCurrent() >= Constants.IntakeConstants.maxCurrentIntake && timer.hasElapsed(.3)){
            return true;
        }
        if(!cone && s_Intake.getPDMCurrent() >= Constants.IntakeConstants.maxCurrentIntake  && timer.hasElapsed(.3) ){
            return true; 
        }
        
        return false; 
    }
}

