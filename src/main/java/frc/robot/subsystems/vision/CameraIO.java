package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {
        public String cameraName = "";
        public boolean connected = false;
        public boolean driverMode = false;
        public byte[] targetData = {};
        public double targetTimeStamp = 0.0;
        public double[] cameraMatrixData = {};
        public double[] distCoeffsData = {};
    }

    public default void updateInputs(CameraIOInputs inputs) {
    }
}
