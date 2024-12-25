package overcharged.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import static overcharged.config.RobotConstants.TAG_SL;
import static overcharged.config.RobotConstants.TAG_T;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.Button;
import overcharged.components.RobotMecanum;
import overcharged.pedroPathing.follower.Follower;
import overcharged.pedroPathing.pathGeneration.Vector;

@Disabled
@TeleOp(name = "servoTest", group = "Test")
public class servoTesting extends LinearOpMode {
    private Servo intakeTiltServo;
    private final static float MIN_SERVO_TICK = 1;
    Button a, b, x, y;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeTiltServo = hardwareMap.get(Servo.class, "intakeTilt");
        long timestamp = 0;
        int posJoy1 = (int) (intakeTiltServo.getPosition() * 255f);

        waitForStart();

        while (opModeIsActive()) {

            long timeStamp = System.currentTimeMillis();

            //calibration
            if (gamepad1.x && Button.BTN_MINUS.canPress4Short(timeStamp)) {
                posJoy1 -= MIN_SERVO_TICK;
            } else if (gamepad1.b && Button.BTN_PLUS.canPress4Short(timeStamp)) {
                posJoy1 += MIN_SERVO_TICK;
            } else if (gamepad1.y && Button.BTN_MAX.canPress(timeStamp)) {
                posJoy1 = 255;
            } else if (gamepad1.a && Button.BTN_MIN.canPress(timeStamp)) {
                posJoy1 = 0;
            } else if (gamepad1.right_stick_button && Button.BTN_MID.canPress(timeStamp)) {
                posJoy1 = 128;
            }
            posJoy1 = Range.clip(posJoy1, 0, 255);
            intakeTiltServo.setPosition(posJoy1 / 255f);

            telemetry.addData("Servo", "servo0");
            telemetry.addData("Adjust", "+: B -: X Max: Y Min: A Mid: RStick");
            telemetry.addData("Position", Integer.toString(posJoy1));
            telemetry.update();
            idle();
        }
    }
}
