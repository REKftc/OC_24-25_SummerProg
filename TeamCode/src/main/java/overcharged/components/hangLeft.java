/*package overcharged.components;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class hangLeft {
    public CRServo hangLeft;

    // Power constants for CRServo
    public static final double POWER_FORWARD = 1;
    public static final double POWER_REVERSE = 0;
    public static final double POWER_STOP = 0.5;

    public hangLeft(HardwareMap hardwareMap) {
        hangLeft = hardwareMap.get(CRServo.class, "hangL");
    }

    public void upLeft() {
        hangLeft.setPower(POWER_FORWARD);
    }

    public void downLeft() {
        hangLeft.setPower(POWER_REVERSE);
    }

    public void stopLeft() {
        hangLeft.setPower(POWER_STOP);
    }
}*/
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
        if (hangLeft == null) {
            throw new IllegalStateException("hangL servo not found in hardware map!");
        }
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

