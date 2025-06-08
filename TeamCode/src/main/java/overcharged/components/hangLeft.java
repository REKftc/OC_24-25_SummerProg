package overcharged.components;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class hangLeft {
    public CRServo hangLeft;

    public static final double POWER_FORWARDL = 1;
    public static final double POWER_REVERSEL = -1;
    public static final double POWER_STOP = 0;

    public hangLeft(HardwareMap hardwareMap) {
        hangLeft = hardwareMap.get(CRServo.class, "hangL");
        /*if (hangLeft == null) {
            throw new IllegalStateException("hangL servo not found");
        }*/
    }

    public void upLeft() {
        hangLeft.setPower(POWER_FORWARDL);
    }

    public void downLeft() {
        hangLeft.setPower(POWER_REVERSEL);
    }

    public void stopLeft() {
        hangLeft.setPower(POWER_STOP);
    }
}

