package overcharged.opmode;
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

@Config
@TeleOp(name="testswitch", group="Teleop")
public class TestSwitch extends OpMode {
    RobotMecanum robot;
    double slowPower = 1;
    long startTime;
    boolean hSlideGoBottom = false;
    boolean intakeOn = false;
    boolean hSlideisOut = false;
    boolean intakeTransfer = false;
    boolean firstLoop = true;
    IntakeMode intakeMode = IntakeMode.OFF;
    public enum IntakeMode {
        IN,
        OUT,
        OFF;
    }
    @Override
    public void init() {
        try {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            robot = new RobotMecanum(this, false, false);
            startTime = System.currentTimeMillis();
            robot.setBulkReadManual();
            //robot.vSlides.vSlidesB.setTargetPositionPIDFCoefficients(21,0,0,0);
        } catch (Exception e) {
            RobotLog.ee(TAG_T, "Teleop init failed: " + e.getMessage());
            telemetry.addData("Init Failed", e.getMessage());
            telemetry.update();
        }
    }
    @Override
    public void loop() {
        telemetry.addData("h limit switch: ", robot.hslides.switchSlideDown.isTouch());
        telemetry.addData("hslideOut", robot.hslides.slideIn());
        telemetry.addData("hslidePower", robot.hslides.getPower());

        telemetry.update();
    }
}
