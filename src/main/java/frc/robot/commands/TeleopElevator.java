package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class TeleopElevator extends CommandBase{
    private Elevator s_Elevator;
    private DoubleSupplier moveVal;

    public TeleopElevator(Elevator s_Elevator, DoubleSupplier moveVal){
        this.s_Elevator = s_Elevator;
        this.moveVal = moveVal;
    }

    public void execute(){
        s_Elevator.move(MathUtil.clamp(moveVal.getAsDouble(), -.1, .1));
    }
}
