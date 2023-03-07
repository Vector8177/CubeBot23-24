package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDs.VectorWave;
import frc.robot.Constants.LEDs.OrangeDot;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.LEDs.Flash;
import frc.robot.Constants.LEDs.LEDMode;

public class LEDs extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private LEDMode mode;

    private GamePiece hpSignal = GamePiece.CONE;

    public LEDs() {
        this.mode = Constants.LEDs.defaultMode;
        this.m_led = new AddressableLED(Constants.LEDs.id);
        this.m_ledBuffer = new AddressableLEDBuffer(Constants.LEDs.length);

        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    @Override
    public void periodic() {
        switch (mode) {
            case RAINBOW:
                rainbow();
                break;
            case VECTORWAVE:
                vectorWave();
                break;
            case ORANGEDOT:
                orangeDot();
                break;
            case PURPLEFLASH:
                flash(276, 89, 91);
                break;
            case YELLOWFLASH:
                flash(58, 100, 100);
                break;
        }
    }

    public void setLEDMode(LEDMode mode) {
        this.mode = mode;
    }

    public void toggleHPSignal() {
        switch (hpSignal) {
            case CONE:
                hpSignal = GamePiece.CUBE;
                setLEDMode(LEDMode.PURPLEFLASH);
                break;
            case CUBE:
                hpSignal = GamePiece.CONE;
                setLEDMode(LEDMode.YELLOWFLASH);
                break;
        }
    }
    /*
     * Flashing of color
     */

    private double brightness = 0;
    private int multiplier = 1;

    private void flash(int h, int s, int v) {
        brightness = MathUtil.clamp(brightness + Flash.speed * multiplier, 0, v);
        if (brightness == v || brightness == 0) {
            multiplier *= -1;
        }

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, h, s, (int) brightness);
        }
    }

    /*
     * Rainbow LED mode
     */

    private int m_rainbowFirstPixelHue = 0;

    private void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
    }

    /*
     * Orange with black wave
     */

    private double m_vectorWaveMiddleIndex = -VectorWave.pauseBetween;

    private void vectorWave() {
        for (int i = 0; i < m_ledBuffer.getLength() + VectorWave.pauseBetween; i++) {
            int value = MathUtil.clamp(
                    (int) ((1 / VectorWave.spread) * (Math.abs(m_vectorWaveMiddleIndex - i)) * VectorWave.v)
                            - VectorWave.length,
                    0,
                    VectorWave.v);

            if (m_ledBuffer.getLength() > i)
                m_ledBuffer.setHSV(i, VectorWave.h, VectorWave.s, value);
        }
        m_vectorWaveMiddleIndex = (m_vectorWaveMiddleIndex + VectorWave.speed) > m_ledBuffer.getLength()
                + VectorWave.pauseBetween ? -VectorWave.pauseBetween
                        : (m_vectorWaveMiddleIndex + VectorWave.speed);
    }

    /*
     * Orange dot
     */

    private double m_orangeDotMiddleIndex = -VectorWave.pauseBetween;

    private void orangeDot() {
        for (int i = 0; i < m_ledBuffer.getLength() + OrangeDot.pauseBetween; i++) {
            int value = MathUtil.clamp(
                    (int) ((1 / OrangeDot.spread) * (Math.abs(m_orangeDotMiddleIndex - i)) * VectorWave.v)
                            - OrangeDot.length,
                    0,
                    VectorWave.v) - VectorWave.v;

            if (m_ledBuffer.getLength() > i)
                m_ledBuffer.setHSV(i, OrangeDot.h, OrangeDot.s, value);
        }
        m_orangeDotMiddleIndex = (m_orangeDotMiddleIndex + OrangeDot.speed) > m_ledBuffer.getLength()
                + OrangeDot.pauseBetween ? -OrangeDot.pauseBetween
                        : (m_orangeDotMiddleIndex + OrangeDot.speed);
    }

}