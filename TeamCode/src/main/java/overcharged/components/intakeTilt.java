package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class intakeTilt {
    public OcServo intakeTilt;
    public static final float INIT = 190f;


    public intakeTilt(HardwareMap hardwareMap) {
        intakeTilt = new OcServo(hardwareMap, "intakeTilt", INIT);
    }
}
