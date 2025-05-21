package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class depoTilt {
    public OcServo depoTilt;
    public static final float INIT = 227f;

    public static final float TRANSFER = 227f;

    public static final float FLAT = 212f;//TODO: This is transfer Sequence

    public static final float SPEC = 180f;//TODO: This is specimen

    public static final float BUCKET = 77f;

    public static final float WALL = 6f;


    public depoTilt(HardwareMap hardwareMap) {
        depoTilt = new OcServo(hardwareMap, "depoTilt", INIT);
    }
    public void setPosition(float pos) {
        depoTilt.setPosition(pos);
    }

    public void setInit() {
        depoTilt.setPosition(INIT);
    }

    public void setTransfer() {
        depoTilt.setPosition(TRANSFER);
    }

    public void setSpec() { //TODO: this is specimen
        depoTilt.setPosition(SPEC);
    }

    public void setOut() { //TODO: this is bucket
        depoTilt.setPosition(BUCKET);
    }

    public void setTranSeq() {
        depoTilt.setPosition(WALL);
    }

    public void setWall() {
        depoTilt.setPosition(WALL);
    }

}
