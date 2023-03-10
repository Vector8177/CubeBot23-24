package frc.lib.util;

public class HSV {
    public int h;
    public int s;
    public int v;

    public HSV(int h, int s, int v) {
        this.h = h;
        this.s = s;
        this.v = v;
    }

    /**
     * Returns an HSV object that converts the
     * Google color picker range (360, 100,100)
     * to WPILib compatible ones.
     * 
     * @param h
     * @param s
     * @param v
     * @return HSV
     */
    public static HSV googleColorPickerHSV(int h, int s, int v) {
        int nH = (int) (((double)h / 360.0) * 179);
        int nS = (int) (((double)s / 100.0) * 255);
        int nV = (int) (((double)v / 100.0) * 255);
        return new HSV(nH, nS, nV);
    }
}
