package frc.robot.subsystems.LEDs.LEDModes;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import frc.robot.Constants.LEDs;

/*
     * Orange dot
     */
public class OrangeDot extends LEDModeBase {
    private double m_orangeDotMiddleIndex = -LEDs.OrangeDot.pauseBetween;

    public OrangeDot(AddressableLEDBuffer m_ledBuffer) {
        super(m_ledBuffer);
    }

    public void execute() {
        for (int i = 0; i < m_ledBuffer.getLength() + LEDs.OrangeDot.pauseBetween; i++) {
            int value = MathUtil.clamp(
                    (int) ((1 / LEDs.OrangeDot.spread) * (Math.abs(m_orangeDotMiddleIndex - i)) * LEDs.OrangeDot.v)
                            - LEDs.OrangeDot.length,
                    0,
                    LEDs.OrangeDot.v) - LEDs.OrangeDot.v;

            if (m_ledBuffer.getLength() > i)
                m_ledBuffer.setHSV(i, LEDs.OrangeDot.h, LEDs.OrangeDot.s, value);
        }
        m_orangeDotMiddleIndex = (m_orangeDotMiddleIndex + LEDs.OrangeDot.speed) > m_ledBuffer.getLength()
                + LEDs.OrangeDot.pauseBetween ? -LEDs.OrangeDot.pauseBetween
                        : (m_orangeDotMiddleIndex + LEDs.OrangeDot.speed);
    }

}
