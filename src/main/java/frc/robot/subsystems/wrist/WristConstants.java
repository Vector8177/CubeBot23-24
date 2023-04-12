package frc.robot.subsystems.wrist;

public final class WristConstants {
    public static final int wristMotorId = 61;
    public static final double maxMotorVoltage = 10.0;

    public enum PIDFFmode {
        WEIGHTED(
                weightedP,
                weightedI,
                weightedD,
                weightedS,
                weightedV,
                weightedA,
                weightedG),
        UNWEIGHTED(
                unweightedP,
                unweightedI,
                unweightedD,
                unweightedS,
                unweightedV,
                unweightedA,
                unweightedG);

        public final double kP;
        public final double kI;
        public final double kD;
        public final double kS;
        public final double kV;
        public final double kA;
        public final double kG;

        private PIDFFmode(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.kG = kG;
        }

    }

    public static double weightedP = 2.8;
    public static double weightedI = 0.0;
    public static double weightedD = 0.2;

    public static double weightedS = 0.274235; // Old Value: 0.4361
    public static double weightedV = 0.67715; // Old Value: 0.79036
    public static double weightedA = 0.020744; // Old Value: 0.0
    public static double weightedG = 0.81416; // Old Value: 0.86416

    public static double unweightedP = 2.0;
    public static double unweightedI = 0.0;
    public static double unweightedD = 0.2;

    public static double unweightedS = 0.11237;
    public static double unweightedV = 0.56387;
    public static double unweightedA = 0.041488;
    public static double unweightedG = 0.76416;

    public static final double motorGearRatio = 1 / 32.0;
    public static final double absoluteEncoderOffset = 5.412927;

    public static final int currentLimit = 40;
}
