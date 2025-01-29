package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class clawBigTilt {
    //public OcServo intakeTilt;
    public OcServo clawBigTilt;
    public VoltageSensor intakeVolt;
    public static final float INIT = 124f;//230f;//90f;//70f;//230f;
    public static final float TRANSFER = 231f;//225f;//233f;//226f;//223f;//201f;//175f;
    public static final float TEMP = 222f;//188f;
    public static final float FLAT = 182f;//161f;//107f;//158f;
    public static final float OUT = 160f;//187f;//159f;//139f DONE
    public static final float WALL = 17f;//52f;
    public static final float BUCKET = 46f;


    public clawBigTilt(HardwareMap hardwareMap) {
        clawBigTilt = new OcServo(hardwareMap, "clawBigTilt", TRANSFER);
        //intakeTilt = hardwareMap.get(OcServo.class, "intakeTilt");
        //intakeVolt = hardwareMap.voltageSensor.iterator().next();
    }
    public void setPosition(float pos){
        clawBigTilt.setPosition(pos);
    }

    public void setInit() { clawBigTilt.setPosition(INIT); }

    public void setTransfer() { clawBigTilt.setPosition(TRANSFER); }

    public void setFlat() { clawBigTilt.setPosition(FLAT); }

    public void setSlides() { clawBigTilt.setPosition(TEMP); }

    public void setOut() { clawBigTilt.setPosition(OUT); }

    public void setWall() { clawBigTilt.setPosition(WALL); }


    public void setBucket() { clawBigTilt.setPosition(BUCKET); }

    //public void getVoltage() { intakeVolt.getVoltage();}
}
