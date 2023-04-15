package frc.robot.subsystems.LEDs.LEDModes;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.VectorTools.util.HSV;
import frc.robot.subsystems.LEDs.LEDConstants;

/*
 * Orange with black wave
 */
public class VectorWave extends LEDModeBase {
    private double vectorWaveMiddleIndex = -LEDConstants.VectorWave.pauseBetween;
    private HSV hsv = LEDConstants.VectorWave.hsv;

    public VectorWave(AddressableLEDBuffer m_ledBuffer) {
        super(m_ledBuffer);
    }

    public void execute() {
        for (int i = 0; i < m_ledBuffer.getLength() / 2 + LEDConstants.VectorWave.pauseBetween; i++) {
            int value =
                    MathUtil.clamp(
                                    (int)
                                                    ((1 / LEDConstants.VectorWave.spread)
                                                            * (Math.abs(vectorWaveMiddleIndex - i))
                                                            * hsv.v)
                                            - LEDConstants.VectorWave.length,
                                    0,
                                    hsv.v - 10)
                            + 10;

            if (m_ledBuffer.getLength() / 2 > i) {
                m_ledBuffer.setHSV(i, hsv.h, hsv.s, value);
                m_ledBuffer.setHSV(m_ledBuffer.getLength() - i - 1, hsv.h, hsv.s, value);
            }
        }
        vectorWaveMiddleIndex =
                (vectorWaveMiddleIndex + LEDConstants.VectorWave.speed)
                                > m_ledBuffer.getLength() + LEDConstants.VectorWave.pauseBetween
                        ? -LEDConstants.VectorWave.pauseBetween
                        : (vectorWaveMiddleIndex + LEDConstants.VectorWave.speed);
    }
}
