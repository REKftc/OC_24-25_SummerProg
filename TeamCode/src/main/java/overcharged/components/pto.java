package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class pto {
    //public OcServo intakeTilt;
    public OcServo pto, pto2;
    //public VoltageSensor intakeVolt;
    public static final float INIT = 74f;
    public static final float INIT2 = 105f;

    public static final float OUT = 118f;
    public static final float OUT2 = 62f;


    public pto(HardwareMap hardwareMap) {
        pto = new OcServo(hardwareMap, "pto", INIT);
        pto2 = new OcServo(hardwareMap, "pto2", INIT2);
    }
    public void setPosition(float pos){
        pto.setPosition(pos);
    }

    public void setInit() { pto.setPosition(INIT);
                            pto2.setPosition(INIT2);
    }

    public void setOut() { pto.setPosition(OUT);
                            pto2.setPosition(OUT2);
    }

}
