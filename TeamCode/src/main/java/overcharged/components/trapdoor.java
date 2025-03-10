package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class trapdoor {

    public OcServo trapdoor;
    public static final float CLOSED = 57f;
    public static final float OPEN = 168f;

    public trapdoor(HardwareMap hardwareMap) {
        trapdoor = new OcServo(hardwareMap, "trapdoor", CLOSED);
    }

    public void setPosition(float pos) { trapdoor.setPosition(pos);}

    public void setInit() { trapdoor.setPosition(CLOSED); }

    public void setOut() { trapdoor.setPosition(OPEN); }
}

