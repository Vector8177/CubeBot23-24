package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
    public static final String leftCameraName =
            "velocityleft"; // Camera to the left (Robot perspective)
    public static final Transform3d leftCameraPosition =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(9.1505),
                            Units.inchesToMeters(9.666),
                            Units.inchesToMeters(31.185)),
                    new Rotation3d(0, Units.degreesToRadians(20), Units.degreesToRadians(5)));

    public static final String rightCameraName =
            "velocityright"; // Camera to the right (Robot perspective)
    public static final Transform3d rightCameraPosition =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(9.1505),
                            Units.inchesToMeters(-9.666),
                            Units.inchesToMeters(31.185)),
                    new Rotation3d(0, Units.degreesToRadians(20), Units.degreesToRadians(-5)));

    public static final class PoseEstimation {
        public interface StandardDeviation {
            Vector<N3> forMeasurement(double distance, int count);
        }

        /**
         * Standard deviations of model states. Increase these numbers to trust your model's state
         * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
         */
        public static final StandardDeviation PHOTON_VISION_STD_DEV =
                (distance, count) -> {
                    double distanceMultiplier = Math.pow(distance - ((count - 1) * 2), 2);
                    double translationalStdDev = (0.05 / (count)) * distanceMultiplier + 0.05;
                    double rotationalStdDev = 0.2 * distanceMultiplier + 0.1;
                    return VecBuilder.fill(translationalStdDev, translationalStdDev, rotationalStdDev);
                };

        public static final double POSE_AMBIGUITY_CUTOFF = .3;

        public static final double POSE_DISTANCE_CUTOFF = FieldConstants.fieldLength / 2;
    }

    public static class FieldConstants {
        public static final double fieldLength = 16.542;
        public static final double fieldWidth = 8.0137;
    }
}
