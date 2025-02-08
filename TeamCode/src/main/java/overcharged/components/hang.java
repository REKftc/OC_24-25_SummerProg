package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class hang {
    public OcServo hang1, hang2;
    public static final float INIT1 = 19f;
    public static final float OUT1 = 205f;

    public static final float INIT2 = 223f;
    public static final float OUT2 = 35f;



    public hang(HardwareMap hardwareMap) {
        hang1 = new OcServo(hardwareMap, "hangL", INIT1);
        hang2 = new OcServo(hardwareMap, "hangR", INIT2);
    }

    public void setPosition(float pos) {
        hang1.setPosition(pos);
        hang2.setPosition(pos);
    }


    public void setInit() {
        hang1.setPosition(INIT1);
        hang2.setPosition(INIT2);
    }

    public void setOut() {
        hang1.setPosition(OUT1);
        hang2.setPosition(OUT2);
    }
}
