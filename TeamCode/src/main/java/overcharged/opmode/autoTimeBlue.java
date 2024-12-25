package overcharged.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import overcharged.components.RobotMecanum;
import overcharged.components.propLocation;
import overcharged.drive.SampleMecanumDrive;
import overcharged.linear.util.WaitLinear;

@Autonomous(name="autoTimeBlue")
public class autoTimeBlue extends LinearOpMode {
    private RobotMecanum robot;
    SampleMecanumDrive drive;
    long startTime;
    MultipleTelemetry telems;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        //try {
            telems = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
            robot = new RobotMecanum(this, true, false);
            WaitLinear lp = new WaitLinear(this);

            waitForStart();
            if (opModeIsActive()) {
                //robot.clearBulkCache();
                startTime = System.currentTimeMillis();
                while(System.currentTimeMillis()-startTime<2500) {
                    robot.driveLeftFront.setPower(0.25f);
                    robot.driveLeftBack.setPower(0.25f);
                    robot.driveRightFront.setPower(0.25f);
                    robot.driveRightBack.setPower(0.25f);
                }
                robot.driveLeftFront.setPower(0f);
                robot.driveLeftBack.setPower(0f);
                robot.driveRightFront.setPower(0f);
                robot.driveRightBack.setPower(0f);
                idle();
                //AutoBody();
            }
        //} catch(Exception e) {
          //  telemetry.addLine("Failed to run.");
     //       telemetry.update();
       // }


    }
    public void AutoBody() throws InterruptedException {
        telemetry.addLine("Running");
        telemetry.addData("Time", System.currentTimeMillis()-startTime);
        telemetry.update();

            //telemetry.addData("Time", System.currentTimeMillis()-startTime);
            //telemetry.update();
        robot.driveLeftFront.setPower(-0.5f);
        robot.driveLeftBack.setPower(-0.5f);
        robot.driveRightFront.setPower(-0.5f);
        robot.driveRightBack.setPower(-0.5f);
        /*if(System.currentTimeMillis()-startTime>10000){
            robot.driveLeftFront.setPower(0f);
            robot.driveLeftBack.setPower(0f);
            robot.driveRightFront.setPower(0f);
            robot.driveRightBack.setPower(0f);
        }

         */
        idle();
    }
}