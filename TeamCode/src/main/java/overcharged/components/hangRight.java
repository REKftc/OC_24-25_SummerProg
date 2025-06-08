/*package overcharged.components;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class hangRight {
    public CRServo hangLeft;

    // Power constants for CRServo
    public static final double POWER_FORWARD = 1;
    public static final double POWER_REVERSE = 0;
    public static final double POWER_STOP = 0.5;

    public hangRight(HardwareMap hardwareMap) {
        hangLeft = hardwareMap.get(CRServo.class, "hangR");
    }

    public void upRight() {
        hangLeft.setPower(POWER_FORWARD);
    }

    public void downRight() {
        hangLeft.setPower(POWER_REVERSE);
    }

    public void stopRight() {
        hangLeft.setPower(POWER_STOP);
    }
}*/
package overcharged.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class hangRight {
    public CRServo hangRight;

    public static final double POWER_FORWARDR = -1;
    public static final double POWER_REVERSER = 1;
    public static final double POWER_STOP = 0;

    public hangRight(HardwareMap hardwareMap) {
        hangRight = hardwareMap.get(CRServo.class, "hangR");
        if (hangRight == null) {
            throw new IllegalStateException("hangR servo not found in hardware map!");
        }
    }

    public void upRight() {
        hangRight.setPower(POWER_FORWARDR);
    }

    public void downRight() {
        hangRight.setPower(POWER_REVERSER);
    }

    public void stopRight() {
        hangRight.setPower(POWER_STOP);
    }
}

