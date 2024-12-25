package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class hang {
    //public OcServo intakeTilt;
    public OcServo hang1, hang2;
    //public VoltageSensor intakeVolt;
    public static final float INIT = 189f;
   // public static final float TRANSFER = 70f;//175f;
   // public static final float SPEC = 171f;//158f;
    public static final float OUT = 49f;


    public hang(HardwareMap hardwareMap) {
        hang1 = new OcServo(hardwareMap, "hang", INIT);
        //intakeTilt = hardwareMap.get(OcServo.class, "intakeTilt");
        //intakeVolt = hardwareMap.voltageSensor.iterator().next();
    }
    public void setPosition(float pos){
        hang1.setPosition(pos);
    }

    public void setInit() { hang1.setPosition(INIT); }

    //public void setTransfer() { intakeTilt.setPosition(TRANSFER); }

   // public void setFlat() { intakeTilt.setPosition(SPEC); }

    public void setOut() { hang1.setPosition(OUT); }

    //public void getVoltage() { intakeVolt.getVoltage();}
}
