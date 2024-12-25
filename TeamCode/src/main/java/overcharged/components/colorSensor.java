package overcharged.components;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.config.RobotConstants;

public class colorSensor {
    public enum Color {
        RED,
        BLUE,
        YELLOW,
        NONE;
    }
    public RevColorSensorV3 colorSensorF;
    public double wavelength;
    private Color color = Color.NONE;
    private float[] rgb = new float[3];
    public colorSensor(HardwareMap hardwareMap) {
        try {
            colorSensorF = hardwareMap.get(RevColorSensorV3.class, "colorSensorF");
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: colorSensorF" + e.getMessage());
        }
    }
    public Color getColor() {
        float hue = getHSV()[0];
        float saturation = getHSV()[0];
        float value = getHSV()[2];
        if(value > 0.35f) {
            if (175 <= hue && hue <= 300) {
                color = Color.BLUE;
            } else if (0 <= hue && hue <= 30) {
                color = Color.RED;
            } else if (51 <= hue && hue <= 120) {
                color = Color.YELLOW;
            }
        }

        else{
            color = Color.NONE;
        }
        return color;
    }
    public float[] getRGB(){
        rgb[0] = colorSensorF.getNormalizedColors().red*255;

        rgb[1] = colorSensorF.getNormalizedColors().green*255;

        rgb[2] = colorSensorF.getNormalizedColors().blue*255;
        return rgb;
    }
    public float[] getHSV() {
        // Normalize RGB values to the range [0, 1]
        float rNorm = colorSensorF.getNormalizedColors().red;
        float gNorm = colorSensorF.getNormalizedColors().green;
        float bNorm = colorSensorF.getNormalizedColors().blue;

        // Calculate Cmax, Cmin, and delta
        float cmax = Math.max(rNorm, Math.max(gNorm, bNorm));
        float cmin = Math.min(rNorm, Math.min(gNorm, bNorm));
        float delta = cmax - cmin;

        // Initialize hue, saturation, and value
        float h = 0;
        float s = 0;
        float v = cmax;

        // Calculate Hue
        if (delta != 0) {
            if (cmax == rNorm) {
                h = (60 * ((gNorm - bNorm) / delta) + 360) % 360;
            } else if (cmax == gNorm) {
                h = (60 * ((bNorm - rNorm) / delta) + 120) % 360;
            } else if (cmax == bNorm) {
                h = (60 * ((rNorm - gNorm) / delta) + 240) % 360;
            }
        }

        // Calculate Saturation
        if (cmax != 0) {
            s = delta / cmax;
        }

        // Return HSV values
        return new float[] { h, s * 100, v * 100 };  // Hue in degrees, Saturation and Value in percentage
    }

}
