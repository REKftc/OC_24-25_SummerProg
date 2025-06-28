package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class claw {
    public OcServo claw;
    public static final float CLOSE = 103f;
    public static final float OPEN = 157f;


    public claw(HardwareMap hardwareMap) {
        claw = new OcServo(hardwareMap, "claw", OPEN);
    }

    public void setPosition(float pos) {
        claw.setPosition(pos);
    }

    public void setClose() {
        claw.setPosition(CLOSE);
    }


    public void setOpen() {
        claw.setPosition(OPEN);
    }


}

