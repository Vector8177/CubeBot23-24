package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class TeleopElevator extends CommandBase {
    private Elevator s_Elevator;
    private DoubleSupplier moveVal;
    private SlewRateLimiter ramp = new SlewRateLimiter(3.0);

    public TeleopElevator(Elevator s_Elevator, DoubleSupplier moveVal) {
        this.s_Elevator = s_Elevator;
        this.moveVal = moveVal;

        addRequirements(s_Elevator);
    }

    @Override
    public void execute() {
        s_Elevator.move(ramp.calculate(MathUtil.clamp(moveVal.getAsDouble(), -.6, 6)));
    }
}
