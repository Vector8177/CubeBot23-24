package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class OuttakeCubeCmd extends CommandBase {
    
    Intake intakeSubsystem; 
    Timer time;

    public OuttakeCubeCmd(Intake intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem; 
        this.time = new Timer();
        
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        System.out.println("OuttakeCubeCmd started"); 
        time.start();
    }
    
    @Override
    public void execute(){
        intakeSubsystem.setMotor(-Constants.IntakeConstants.coneIntakeSpeed);
    }

    @Override
    public void end(boolean interrupted){
        time.stop();
        time.reset();
        intakeSubsystem.setMotor(0);
        System.out.println("OuttakeCubeCmd ended"); 
    }

    @Override
    public boolean isFinished(){
      
        
        return time.hasElapsed(.5);
    }
}

