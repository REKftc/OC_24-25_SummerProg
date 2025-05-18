package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_SL;
import static overcharged.config.RobotConstants.TAG_T;

import android.transition.Slide;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.Button;
import overcharged.components.RobotMecanum;
import overcharged.components.colorSensor;
import overcharged.components.hang;
import overcharged.components.hslides;
import overcharged.components.vSlides;

@Config
@TeleOp(name="slide test", group="Test")
public class slideTuner extends OpMode {
    private RobotMecanum robot;

    public static float kP = 0.07f;
    public static int target = 0;

    public void loop(){
        robot.vSlides.setUseSquID(true, target, 1f);
        robot.vSlides.setKp(kP);
        robot.vSlides.update();
    }

    public void init(){
        robot = new RobotMecanum(this, false, false);
        telemetry.addData("target: ", robot.vSlides.getTarget());
        telemetry.addData("position: ", robot.vSlides.getCurrentPosition());
    }
}
