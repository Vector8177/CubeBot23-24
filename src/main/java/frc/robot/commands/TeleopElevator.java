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
        double deadbandController = Math.abs(moveVal.getAsDouble()) > .1 ? -moveVal.getAsDouble() : 0;
        s_Elevator.setPosition((s_Elevator.getPosition() + deadbandController*.2));
    }
}
