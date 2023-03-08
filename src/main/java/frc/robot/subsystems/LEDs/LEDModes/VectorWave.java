package frc.robot.subsystems.LEDs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.LEDs;

/*
     * Orange with black wave
     */
public class VectorWave implements LEDModeBase {
    private double vectorWaveMiddleIndex = -LEDs.VectorWave.pauseBetween;

    public void execute(AddressableLEDBuffer m_ledBuffer) {
        for (int i = 0; i < m_ledBuffer.getLength() + LEDs.VectorWave.pauseBetween; i++) {
            int value = MathUtil.clamp(
                    (int) ((1 / LEDs.VectorWave.spread) * (Math.abs(vectorWaveMiddleIndex - i)) * LEDs.VectorWave.v)
                            - LEDs.VectorWave.length,
                    0,
                    LEDs.VectorWave.v);

            if (m_ledBuffer.getLength() > i)
                m_ledBuffer.setHSV(i, LEDs.VectorWave.h, LEDs.VectorWave.s, value);
        }
        vectorWaveMiddleIndex = (vectorWaveMiddleIndex + LEDs.VectorWave.speed) > m_ledBuffer.getLength()
                + LEDs.VectorWave.pauseBetween ? -LEDs.VectorWave.pauseBetween
                        : (vectorWaveMiddleIndex + LEDs.VectorWave.speed);
    }
}
