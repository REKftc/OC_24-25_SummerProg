package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class hang {
    public OcServo hang1, hang2;
    public static final float INIT = 189f;
    public static final float OUT = 49f;


    public hang(HardwareMap hardwareMap) {
        //hang1 = new OcServo(hardwareMap, "hangL", INIT);
        hang2 = new OcServo(hardwareMap, "hangR", INIT);
    }

    public void setPosition(float pos) {
        //hang1.setPosition(pos);
        hang2.setPosition(pos);
    }


    public void setInit() {
        //hang1.setPosition(INIT);
        hang2.setPosition(INIT);
    }

    public void setOut() {
        //hang1.setPosition(OUT);
        hang2.setPosition(INIT);
    }
}
