package frc.robot.subsystems.LEDs.LEDModes;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.lib.util.HSV;
import frc.robot.Constants.LEDs;

/*
     * Orange with black wave
     */
public class VectorWave extends LEDModeBase {
    private double vectorWaveMiddleIndex = -LEDs.VectorWave.pauseBetween;
    private HSV hsv = LEDs.VectorWave.hsv;

    public VectorWave(AddressableLEDBuffer m_ledBuffer) {
        super(m_ledBuffer);
    }

    public void execute() {
        for (int i = 0; i < m_ledBuffer.getLength() + LEDs.VectorWave.pauseBetween; i++) {
            int value = MathUtil.clamp(
                    (int) ((1 / LEDs.VectorWave.spread) * (Math.abs(vectorWaveMiddleIndex - i)) * hsv.v)
                            - LEDs.VectorWave.length,
                    0,
                    hsv.v);

            if (m_ledBuffer.getLength() > i)
                m_ledBuffer.setHSV(i, hsv.h, hsv.s, value);
        }
        vectorWaveMiddleIndex = (vectorWaveMiddleIndex + LEDs.VectorWave.speed) > m_ledBuffer.getLength()
                + LEDs.VectorWave.pauseBetween ? -LEDs.VectorWave.pauseBetween
                        : (vectorWaveMiddleIndex + LEDs.VectorWave.speed);
    }
}
