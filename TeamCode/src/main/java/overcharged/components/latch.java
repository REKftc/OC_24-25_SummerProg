package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class latch {
    //public OcServo intakeTilt;
    public OcServo latch;
    //public VoltageSensor intakeVolt;
    public static final float INIT = 137f;//206f;
    // public static final float TRANSFERL = 70f;//175f;
    // public static final float SPECR = 171f;//158f;
    public static final float OUT = 204f;//237f;


    public latch(HardwareMap hardwareMap) {
        latch = new OcServo(hardwareMap, "latch", INIT);
        //intakeTilt = hardwareMap.get(OcServo.class, "intakeTilt");
        //intakeVolt = hardwareMap.voltageSensor.iterator().next();
    }
    public void setPosition(float pos){
        latch.setPosition(pos);
    }

    public void setInit() { latch.setPosition(INIT); }

    //public void setTransfer() { intakeTilt.setPosition(TRANSFERL); }

    // public void setFlat() { intakeTilt.setPosition(SPECR); }

    public void setOut() { latch.setPosition(OUT); }

    //public void getVoltage() { intakeVolt.getVoltage();}
}

