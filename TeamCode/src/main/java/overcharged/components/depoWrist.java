package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class depoWrist {
    public OcServo depoWrist;
    public static final float IN = 213f;//124f;//125f;
    public static final float OUT = 57f;//198f;
    public static final float TRANSFER = 224f;
    public static final float SPEC = 173f;
    public static final float FLAT = 204f;
    public static final float BUCKET = 40f;
    public static final float WALL = 70f;

    public depoWrist(HardwareMap hardwareMap) {
        depoWrist = new OcServo(hardwareMap, "depoWrist", TRANSFER);
    }
    public void setPosition(float pos){
        depoWrist.setPosition(pos);
    }

    public void setIn() { depoWrist.setPosition(IN); }

    public void setOut() { depoWrist.setPosition(OUT); }

    public void setTransfer() { depoWrist.setPosition(TRANSFER); }

    public void setSpecimen() { depoWrist.setPosition(SPEC); }

    public void setFlat() { depoWrist.setPosition(FLAT); }

    public void setBucket() { depoWrist.setPosition(BUCKET); }

    public void setWall() { depoWrist.setPosition(WALL); }
}
