package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class depoTilt {
    public OcServo depoTiltL, depoTiltR;
    public static final float INIT = 30f;


    public depoTilt(HardwareMap hardwareMap) {
        depoTiltL = new OcServo(hardwareMap, "depoTiltL", INIT);
        depoTiltR = new OcServo(hardwareMap, "depoTiltR", INIT);
    }
}
