package frc.robot.subsystems.intake;

public final class IntakeConstants {
    public static final int motorId = 60;

    public static final int pdpChannel = 2; // update number later

    public static final double stoppedRPMThreshold = 1;

    public static final double coneIntakeSpeed = 8;
    public static final double cubeIntakeSpeed = 5;

    public static final double coneOuttakeSpeed = 3;
    public static final double coneShootSpeed = 12;
    public static final double cubeOuttakeSpeed = 5;

    public static final int currentLimit = 30;

    public enum EjectSpeed {
        CUBEFAST(18),
        CUBENORMAL(6),
        CONEFAST(18),
        CONENORMAL(4),
        CONESLOW(2),
        CONEINTAKE(8);

        public final double speed;

        EjectSpeed(double speed) {
            this.speed = speed;
        }
    }
}
