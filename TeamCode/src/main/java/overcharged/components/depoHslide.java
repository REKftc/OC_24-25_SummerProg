package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class depoHslide {
    //public OcServo intakeTilt;
    public OcServo depoHslide;
    //public VoltageSensor intakeVolt;
    public static final float INIT = 74f;
    public static final float MID = 60f;
    //public static final float TRANSFERL  = 20f;
    // public static final float TRANSFERL = 70f;//175f;
    // public static final float SPECR = 171f;//158f;
    public static final float OUT = 174f;


    public depoHslide(HardwareMap hardwareMap) {
        depoHslide = new OcServo(hardwareMap, "depoHslide", INIT);
        //intakeTilt = hardwareMap.get(OcServo.class, "intakeTilt");
        //intakeVolt = hardwareMap.voltageSensor.iterator().next();
    }
    public void setPosition(float pos){
        depoHslide.setPosition(pos);
    }

    public void setInit() { depoHslide.setPosition(INIT); }

    public void setMid() { depoHslide.setPosition(MID); }
   // public void setTransfer() { depoHslide.setPosition(TRANSFERL); }

    //public void setTransfer() { intakeTilt.setPosition(TRANSFERL); }

    // public void setFlat() { intakeTilt.setPosition(SPECR); }

    public void setOut() { depoHslide.setPosition(OUT); }

    //public void getVoltage() { intakeVolt.getVoltage();}
}


