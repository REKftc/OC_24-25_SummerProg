package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class depoHslide {
    public OcServo depoHslide;
    public static final float INIT = 160f;


    public depoHslide(HardwareMap hardwareMap) {
        depoHslide = new OcServo(hardwareMap, "depoHslide", INIT);
    }
}


