package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double currentPosition = 0.0;

        // Left elevator motor
        public double leftAppliedVolts = 0.0;
        public double[] leftCurrentAmps = new double[] {};
        public double[] leftTempCelcius = new double[] {};

        // Right elevator motor
        public double rightAppliedVolts = 0.0;
        public double[] rightCurrentAmps = new double[] {};
        public double[] rightTempCelcius = new double[] {};
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void resetEncoder() {}

    public default void move(double voltage) {}
}
