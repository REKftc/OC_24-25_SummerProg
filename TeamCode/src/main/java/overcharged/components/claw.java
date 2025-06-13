package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class claw {
    //public OcServo intakeTilt;
    public OcServo claw;
    //public VoltageSensor intakeVolt;
    public static final float INIT = 157f;
    // public static final float TRANSFERL = 70f;//175f;
    public static final float SPECCLOSE = 106f;//158f;
    public static final float CLOSE = 104f;//104f;//105f;
    public static final float SPEC = 98f;
    public static final float MORECLOSE = 96f;
    public static final float HALF_CLOSE = 130f;//257f;//255f;
    public static final float OPEN = 157f;//149f;
    public static final float BIG = 206f;


    public claw(HardwareMap hardwareMap) {
        claw = new OcServo(hardwareMap, "claw", OPEN);
        //intakeTilt = hardwareMap.get(OcServo.class, "intakeTilt");
        //intakeVolt = hardwareMap.voltageSensor.iterator().next();
    }
    public void setPosition(float pos){
        claw.setPosition(pos);
    }

    public void setInit() { claw.setPosition(INIT); }

    public void setClose() { claw.setPosition(CLOSE); }

    public void setSpec() {claw.setPosition(SPEC);}

    public void setHalfClose() {claw.setPosition(HALF_CLOSE);}

    public void setMoreClose() {claw.setPosition(MORECLOSE);}

    public void setSpecClose() { claw.setPosition(SPECCLOSE); }

    public void setOpen() { claw.setPosition(OPEN); }

    public void setBig() {claw.setPosition(BIG);}

    //public void getVoltage() { intakeVolt.getVoltage();}
}

