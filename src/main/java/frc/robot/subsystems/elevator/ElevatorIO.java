package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs
    {
        public double currentPosition = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs)
    {

    }
}
