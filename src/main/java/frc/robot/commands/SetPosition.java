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

        }
    }
}
