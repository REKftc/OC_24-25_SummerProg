package overcharged.components;

import static overcharged.config.RobotConstants.TAG_H;
import static overcharged.config.RobotConstants.TAG_SL;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

import overcharged.config.RobotConstants;

public class hslides {

    public final OcMotorEx hslides;
    //public final OcMotorEx hslidesR;
    public OcSwitch switchSlideDown;
    public final List<OcSwitch> switchs = new ArrayList<>();

    public double start;

    public static int autoLevel = 0;

    public static final int START = 0;
    public static final int SMALL = 180;
    public static final int PRESET1 = 230;
    public static final int PRESET2 = 370;
    public static final int PRESET3 = 480;
    public static final int OUT = 570;

    public static double p = 18;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public hslides(HardwareMap hardwareMap) {
        hslides = new OcMotorEx(hardwareMap, "hslides", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        // hslidesL = new OcMotorEx(hardwareMap, "hslidesL", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        //hslidesR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //hslidesL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hslides.setTargetPositionPIDFCoefficients(p, i, d, f);
        //hslidesL.setTargetPositionPIDFCoefficients(p, i, d, f);

        String missing = "";
        int numberMissing = 0;
        OcMotorEx slideL = null;
        OcSwitch lswitch2 = null;
        try {
            lswitch2 = new OcSwitch(hardwareMap, "hlimitswitch", true);
            boolean isSwitchNull = switchSlideDown == null ? true : false;
            switchs.add(lswitch2);
            RobotLog.ii(TAG_H, "limitSwitch(isNull)? " + isSwitchNull);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R, "missing: limitSwitch " + e.getMessage());
            missing = missing + ", hlimitswitch";
            numberMissing++;
            boolean isSwitchNull = switchSlideDown == null ? true : false;
            RobotLog.ii(TAG_H, "limitSwitch(catch)? " + isSwitchNull);
        }
        this.switchSlideDown = lswitch2;
        start = hslides.getCurrentPosition();
        // start = hslidesL.getCurrentPosition();

        RobotLog.ii(TAG_SL, "Initialized the Slide component numberMissing=" + numberMissing + " missing=" + missing);
    }



    public void moveEncoderTo(int pos, float power) {
        hslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //hslidesL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hslides.setTargetPosition(pos);
        //  hslidesL.setTargetPosition(pos);
        hslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // hslidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //hslidesR.setPower(power);
        hslides.setPower(power);
    }

    public boolean slideIn() {
        return switchSlideDown.isTouch() && hslides.getCurrentPosition() <= start;
    }

    public void in(){
        hslides.setPower(-0.7f);
        //hslidesR.setPower(-0.7f);
    }
    public void inAuto(){
        hslides.setPower(-0.9f);
        //hslidesR.setPower(-0.9f);
    }

    public void setPower(float power)
    {
        int cnt = 1;
        //try {
        if (hslides != null) {
            RobotLog.ii(TAG_SL, "Set slide left motor power to " + power);
            hslides.setPower(power);
            if (power == 0f) {
                hslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else {
            RobotLog.ii(TAG_SL, "Not setting power for motorL");
        }
    }

    public void reset(OcMotorEx motor) {
        motor.setPower(0f);
        motor.setPower(0f);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.resetPosition();
    }
    public float getPower() {
        return hslides.getPower();
    }


    public void forceStop() {
        //if (prev_state != State.STOP) {
        RobotLog.ii(TAG_SL, "Force stop the Slide component");
        hslides.setPower(0f);
    }
}
