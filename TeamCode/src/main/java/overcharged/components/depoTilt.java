package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class depoTilt {
    public OcServo depoTiltL, depoTiltR;
    public static final float INIT = 30f;

    public static final float TRANSFER = 30f;

    public static final float FLAT = 212f;//TODO: This is transfer Sequence

    public static final float SPEC = 76f;//TODO: This is specimen

    public static final float BUCKET = 186f;

    public static final float WALL = 235f;

    public static final float WALLAUTO = 7f;


    public depoTilt(HardwareMap hardwareMap) {
        depoTiltL = new OcServo(hardwareMap, "depoTiltL", INIT);
        depoTiltR = new OcServo(hardwareMap, "depoTiltR", INIT);
    }
    public void setPosition(float pos) {
        depoTiltL.setPosition(pos);
        depoTiltR.setPosition(pos);
    }

    public void setInit() {
        depoTiltL.setPosition(INIT);
        depoTiltR.setPosition(INIT);
    }

    public void setTransfer() {
        depoTiltL.setPosition(TRANSFER);
        depoTiltR.setPosition(TRANSFER);
    }

    public void setSpec() { //TODO: this is specimen
        depoTiltL.setPosition(SPEC);
        depoTiltR.setPosition(SPEC);
    }

    public void setOut() { //TODO: this is bucket
        depoTiltL.setPosition(BUCKET);
        depoTiltR.setPosition(BUCKET);
    }

    public void setTranSeq() {
        depoTiltL.setPosition(WALL);
        depoTiltR.setPosition(WALL);
    }

    public void setWall() {
        depoTiltL.setPosition(WALL);
        depoTiltR.setPosition(WALL);
    }

    public void setWallAuto() {
        depoTiltL.setPosition(WALLAUTO);
        depoTiltR.setPosition(WALL);
    }

}
