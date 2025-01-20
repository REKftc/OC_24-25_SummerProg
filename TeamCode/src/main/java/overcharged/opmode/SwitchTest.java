package overcharged.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;
import java.util.List;

import overcharged.components.RobotMecanum;

//import revAmped.components.Button;

@TeleOp(name = "SwitchTest", group = "Test")
//@Disabled
public class SwitchTest extends LinearOpMode {
    RobotMecanum robot;
    private DigitalChannel digital;
   // private OcServo intakeTilt;

    @Override
    public void runOpMode() throws InterruptedException {
//        List<DigitalChannel> test = hardwareMap.getAll(DigitalChannel.class);
//        for (DigitalChannel temp:test
//             ) {
//            telemetry.addData("value in list",temp.getState());
//        }


        digital = hardwareMap.get(DigitalChannel.class, "hlimitswitch");
        robot = new RobotMecanum(this, false, false);
      //  intakeTilt = new OcServo(hardwareMap, "intakeTilt", 100f);
        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();
            robot.intakeTilt.setPosition(100f);
            telemetry.addData("limitswitch", digital.getState());

            telemetry.update();
            idle();
        }
    }
}
