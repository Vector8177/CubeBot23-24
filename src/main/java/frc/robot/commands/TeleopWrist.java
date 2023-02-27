package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class TeleopWrist extends CommandBase{
    private Intake s_Intake;
    private DoubleSupplier moveVal;

    public TeleopWrist(Intake s_Intake,  DoubleSupplier moveVal){
        this.s_Intake = s_Intake; 
        this.moveVal = moveVal;

        addRequirements(s_Intake);
    }

    public void execute(){
        s_Intake.move(MathUtil.clamp(moveVal.getAsDouble(), -0.6, 0.6));
    }
}
