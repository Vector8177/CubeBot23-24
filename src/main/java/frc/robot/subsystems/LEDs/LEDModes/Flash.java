package frc.robot.subsystems.LEDs.LEDModes;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import frc.robot.Constants.LEDs;

public class Flash extends LEDModeBase {
    private double brightness = 0;
    private int multiplier = 1;

    private int h;
    private int s;
    private int v;

    public Flash(AddressableLEDBuffer m_ledBuffer, int h, int s, int v) {
        super(m_ledBuffer);

        this.h = h;
        this.s = s;
        this.v = v;
    }

    public void execute() {
        brightness = MathUtil.clamp(brightness + LEDs.Flash.speed * multiplier, 0, v);
        if (brightness == v || brightness == 0) {
            multiplier *= -1;
        }

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, h, s, (int) brightness);
        }
    }
}
