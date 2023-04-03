package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public double voltage = 0.0;
        public double velocity = 0.0;
        public double current = 0.0;
        public double absoluteEncoderPosition = 0.0;

    }

    public default void updateInputs(WristIOInputs inputs) {

    }

    public default void setVoltage(double speed) {

    }

    public default void resetRelativeEncoder() {

    }

}
