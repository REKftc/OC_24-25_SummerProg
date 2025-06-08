package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class hangRelease {
    public OcServo hangRelease;
    public static final float INIT = 128f;
    public static final float OUT = 200f;


    public hangRelease(HardwareMap hardwareMap) {
        hangRelease = new OcServo(hardwareMap, "hangRelease", INIT);
    }

    public void setPosition(float pos) {
        hangRelease.setPosition(pos);
    }


    public void setInit() {
        hangRelease.setPosition(INIT);
    }

    public void setOut() {
        hangRelease.setPosition(OUT);
    }
}
