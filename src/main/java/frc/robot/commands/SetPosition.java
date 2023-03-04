package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class SetPosition extends CommandBase{
    public SetPosition(Wrist s_Wrist, Elevator s_Elevator, Position position){
        addRequirements(s_Elevator, s_Wrist);

        switch(position){
            case CUBE2:
                s_Wrist.setWristPosition(0);
                s_Elevator.setPosition(16);
                break;
            case CUBE3:
                s_Wrist.setWristPosition(0);
                s_Elevator.setPosition(35);
                break;
            case CONE2:
                s_Wrist.setWristPosition(23);
                s_Elevator.setPosition(23);
                break;
            case CONE3:
                s_Wrist.setWristPosition(1.3963);
                s_Elevator.setPosition(35.1);
                break;
            case BOTTOM:
                s_Wrist.setWristPosition(0.5236);
                s_Elevator.setPosition(0);
                break;
            case STANDBY:
                s_Wrist.setWristPosition(1.3963);
                s_Elevator.setPosition(0);
                break;
            
        }
    }
}
