package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class depoHslide {
    public OcServo depoHslide;
    public static final float INIT = 183f;
    public static final float MID = 60f;
    public static final float TRANSFER = 183f;
    public static final float OUT = 100f;


    public depoHslide(HardwareMap hardwareMap) {
        depoHslide = new OcServo(hardwareMap, "depoHslide", TRANSFER);
        //intakeTilt = hardwareMap.get(OcServo.class, "intakeTilt");
        //intakeVolt = hardwareMap.voltageSensor.iterator().next();
    }
    public void setPosition(float pos){
        depoHslide.setPosition(pos);
    }

    public void setInit() { depoHslide.setPosition(INIT); }

    public void setMid() { depoHslide.setPosition(MID); }

    public void setTransfer() { depoHslide.setPosition(TRANSFER); }

    public void setOut() { depoHslide.setPosition(OUT); }

}


