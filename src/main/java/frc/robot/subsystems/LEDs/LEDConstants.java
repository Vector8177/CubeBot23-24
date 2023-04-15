package frc.robot.subsystems.LEDs;

import frc.VectorTools.util.HSV;

public final class LEDConstants {
    public static final int id = 9;
    public static final int length = 18;

    public static final LEDMode defaultMode = LEDMode.VECTORWAVE;

    public static final class Flash {
        public static final double speed = 5;
    }

    public static final class VectorWave {
        public static final HSV hsv = HSV.googleColorPickerHSV(8, 100, 100);

        public static final int pauseBetween = 10;
        public static final int length = 30;
        public static final double spread = 4;
        public static final double speed = .25;
    }

    public static final class OrangeDot {
        public static final HSV hsv = HSV.googleColorPickerHSV(20, 99, 100);

        public static final int pauseBetween = 10;
        public static final int length = 0;
        public static final double spread = 2;
        public static final double speed = .5;
    }

    public enum LEDMode {
        ORANGEDOT("Orange Dot"),
        VECTORWAVE("Vector Wave"),
        RAINBOW("Rainbow"),
        PURPLEFLASH("Purple Flash"),
        YELLOWFLASH("Yellow Flash"),
        GREENFLASH("Green Flash"),
        REDFLASH("Red Flash");

        private String name;

        private LEDMode(String name) {
            this.name = name;
        }

        @Override
        public String toString() {
            return name;
        }
    }
}
