package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class pto {
    //public OcServo intakeTilt;
    public OcServo pto;
    //public VoltageSensor intakeVolt;
    public static final float INIT = 74f;
    public static final float OUT = 120f;


    public pto(HardwareMap hardwareMap) {
        pto = new OcServo(hardwareMap, "pto", INIT);
    }
    public void setPosition(float pos){
        pto.setPosition(pos);
    }

    public void setInit() { pto.setPosition(INIT); }

    public void setOut() { pto.setPosition(OUT); }

}
