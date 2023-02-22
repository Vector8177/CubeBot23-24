package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ScoreGamePiece extends CommandBase {
    private Elevator s_Elevator;
    private Constants.SEGMENT position;

    public ScoreGamePiece(Elevator s_Elevator, int level, boolean coneMode){
        this.s_Elevator = s_Elevator;

        position = Constants.SEGMENT.getSegment(level, coneMode);
        
        addRequirements(s_Elevator);
    }

    public void execute(){
        s_Elevator.raise(position.getValue());
        if(s_Elevator.reachedSetpoint(position.getValue())){
            
        }
    }
}
