package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class smallHang {
    public OcServo smallHang;
    public static final float INIT = 189f;
    public static final float OUT = 49f;


    public smallHang(HardwareMap hardwareMap) {
        smallHang = new OcServo(hardwareMap, "smallHang", INIT);
    }

    public void setPosition(float pos) {
        smallHang.setPosition(pos);
    }


    public void setInit() {
        smallHang.setPosition(INIT);
    }

    public void setOut() {
        smallHang.setPosition(OUT);
    }
}
