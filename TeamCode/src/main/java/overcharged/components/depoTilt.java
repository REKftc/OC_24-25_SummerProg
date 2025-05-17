package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class depoTilt {
    public OcServo depoLeftTilt, depoRightTilt;
    public static final float INITL = 104f;
    public static final float INITR = 104f;

    public static final float TRANSFERL = 197f;
    public static final float TRANSFERR = 197f;

    public static final float FLATL = 212f;//TODO: This is transfer Sequence
    public static final float FLATR = 212f;

    public static final float SPECL = 141f;//TODO: This is specimen
    public static final float SPECR = 141f;

    public static final float BUCKETL = 212f;
    public static final float BUCKETR = 212f;

    public static final float WALLL = 190f;
    public static final float WALLR = 190f;


    public depoTilt(HardwareMap hardwareMap) {
        depoLeftTilt = new OcServo(hardwareMap, "clawLeftTilt", TRANSFERL);
        depoRightTilt = new OcServo(hardwareMap, "clawRightTilt", TRANSFERL);
    }
    public void setPosition(float pos) {
        depoLeftTilt.setPosition(pos);
        depoLeftTilt.setPosition(pos);
    }

    public void setInit() {
        depoLeftTilt.setPosition(INITR);
        depoLeftTilt.setPosition(INITR);
    }

    public void setTransfer() {
        depoLeftTilt.setPosition(INITR);
        depoLeftTilt.setPosition(INITR);
    }

    public void setFlat() {
        depoLeftTilt.setPosition(INITR);
        depoLeftTilt.setPosition(INITR);
    }

    public void setOut() { //TODO: this is bucket
        depoLeftTilt.setPosition(INITR);
        depoLeftTilt.setPosition(INITR);
    }

    public void setTranSeq() {
        depoLeftTilt.setPosition(INITR);
        depoLeftTilt.setPosition(INITR);
    }

    public void setWall() {
        depoLeftTilt.setPosition(INITR);
        depoLeftTilt.setPosition(INITR);
    }

}
