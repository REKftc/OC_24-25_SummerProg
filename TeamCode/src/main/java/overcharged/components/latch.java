package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class latch {
    public OcServo latch;
    public static final float INIT = 137f;



    public latch(HardwareMap hardwareMap) {
        latch = new OcServo(hardwareMap, "latch", INIT);

    }
}

