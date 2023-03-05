package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Position;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class SetPosition extends CommandBase{
    Wrist s_Wrist;
    Elevator s_Elevator;
    Position position;
    GamePiece gamePiece;

    public SetPosition(Wrist s_Wrist, Elevator s_Elevator, Position position, GamePiece gamePiece){
        this.s_Wrist = s_Wrist;
        this.s_Elevator = s_Elevator;
        this.position = position;
        this.gamePiece = gamePiece;

        addRequirements(s_Elevator, s_Wrist);
        }

        @Override
        public void execute(){
            switch(position){
                case HIGH:
                    if(gamePiece == GamePiece.CONE){
                        s_Wrist.setWristPosition(0);
                        s_Elevator.setPosition(35);
                    } else if(gamePiece == GamePiece.CUBE){
                        s_Wrist.setWristPosition(1.55);
                        s_Elevator.setPosition(35.1);
                    }
                    break;
    
                case MID:
                    if(gamePiece == GamePiece.CONE){
                        s_Wrist.setWristPosition(0);
                        s_Elevator.setPosition(23);
                    } else if(gamePiece == GamePiece.CUBE){
                        s_Wrist.setWristPosition(1.427);
                        s_Elevator.setPosition(16.5);
                    }
                    break;
    
                case LOW:
                    s_Wrist.setWristPosition(0.5236);
                    s_Elevator.setPosition(0);
                    break;
                    
                case STANDBY:
                    s_Wrist.setWristPosition(1.3963);
                    s_Elevator.setPosition(0);
                    break;
        }

        
    }
    @Override
        public boolean isFinished(){
            return true;
        }
}
