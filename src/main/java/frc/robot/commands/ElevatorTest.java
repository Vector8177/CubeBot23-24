package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorTest extends CommandBase {
    DoubleSupplier moveVal;
    Elevator s_Elevator;
    public ElevatorTest(Elevator s_Elevator, DoubleSupplier moveVal){
        this.s_Elevator = s_Elevator;
        this.moveVal = moveVal;

        addRequirements(s_Elevator);
    }

    public void execute(){
        s_Elevator.setPosition(moveVal.getAsDouble());
    }

    public boolean isFinished(){
        return s_Elevator.reachedSetpoint(moveVal.getAsDouble());
    }
}
