package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.pathgen.BezierPoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.RobotMecanum;
import overcharged.components.colorSensor;
import overcharged.components.hslides;
import overcharged.components.vSlides;
import overcharged.drive.SampleMecanumDrive;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.util.Timer;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import overcharged.pedroPathing.constants.FConstants;
import overcharged.pedroPathing.constants.LConstants;

@Autonomous(name = "red bucket new", group = "0Autonomous")
public class autoredbuckettest extends OpMode {

    // Init
    private RobotMecanum robot;
    private DigitalChannel hlimitswitch;
    private DigitalChannel vlimitswitch;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SampleMecanumDrive drive;
    MultipleTelemetry telems;
    private ElapsedTime pathTimer;

    private int pathState;

    private Follower follower;

    boolean vslideGoBottom = false;
    boolean hSlideGoBottom = false;
    boolean in = false;

    //TODO: poses
    private Pose startPose = new Pose(137, 31, Math.toRadians(90));
    private Point bucketScore;
    private Pose beforeBlock, beforeBlock2;

    private Path toBlock, backBucket, toBlock2;

    private PathChain initScore;

    public void buildPoses() {
        bucketScore = new Point(128.7, 12, Point.CARTESIAN);
        beforeBlock = new Pose(109, 15, Math.toRadians(145));
        beforeBlock2 = new Pose(110, 15, Math.toRadians(180));
    }

    //TODO: here are where the paths are defined
    public void buildPaths() {
        initScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose),bucketScore))
                .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(156))
                .build();

        toBlock = new Path(new BezierLine(bucketScore, new Point(beforeBlock)));
        toBlock2 = new Path(new BezierLine(bucketScore, new Point(beforeBlock2)));

        backBucket = new Path(new BezierLine(new Point(beforeBlock), bucketScore));
        backBucket.setLinearHeadingInterpolation(beforeBlock.getHeading(), Math.toRadians(156));
    }


    // TODO: HERE IS WHERE THE MAIN PATH IS
    // Main pathing
    public void autoPath() {
        switch (pathState) {
            case 10: // start
                robot.intakeTilt.setTransfer();
                follower.followPath(initScore, true);
                follower.setMaxPower(0.8f);
                setPathState(102);
                break;
            case 102:
                if(pathTimer.milliseconds()>90){
                    robot.vSlides.moveEncoderTo(robot.vSlides.autohigh1, 1f);
                    setPathState(11);
                }
                break;
            case 11:
                if(pathTimer.milliseconds()>150){
                    robot.latch.setOut();
                    robot.hslides.moveEncoderTo(robot.hslides.SMALL, 0.7f);
                    in = false;
                    robot.depoTilt.setOut();
                    robot.depoHslide.setOut();
                    setPathState(13);
                }
                break;
            case 13:
                if ((follower.getPose().getX() > (bucketScore.getX() - 1.5) && follower.getPose().getY() < (bucketScore.getY() + 1.5) && Math.abs(robot.vSlides.vSlidesL.getCurrentPosition()-660) < 16) && pathTimer.milliseconds()>800 || pathTimer.milliseconds()>1200) {
                    setPathState(14);
                }
                break;
            case 14:
                if(pathTimer.milliseconds()>200) {
                    robot.claw.setOpen();
                    robot.intakeTilt.setOut();
                    robot.intake.in();
                    setPathState(15);
                }
                break;
            case 15:
                if(pathTimer.milliseconds()>300){
                    robot.depoTilt.setTransfer();
                    robot.depoHslide.setTransfer();
                    follower.followPath(toBlock, true);
                    setPathState(16);
                }
                break;
            case 16:
                if(pathTimer.milliseconds() > 150) {
                    vslideGoBottom = true;
                    in = false;
                    robot.hslides.moveEncoderTo(robot.hslides.PRESET3, 0.8f);
                    setPathState(161);
                }
                break;
            case 161:
                if(follower.getCurrentTValue() > 0.93){
                    setPathState(17);
                }
                break;
            case 17:
                if(robot.sensorF.getColor() == colorSensor.Color.YELLOW){
                    setPathState(18);
                }
                else if(robot.sensorF.getColor() == colorSensor.Color.NONE && pathTimer.milliseconds()<1900 && robot.hslides.hslides.getCurrentPosition() < 590) {
                    robot.hslides.moveEncoderTo(robot.hslides.hslides.getCurrentPosition() + 150, 1f);
                }
                break;

            case 18:
            if (!follower.isBusy()){
                robot.intakeTilt.setTransfer();
                robot.intake.in();
            }
            if(pathTimer.milliseconds() > 200) {
                hSlideGoBottom = true;
                follower.followPath(backBucket, true);
                setPathState(19);
            }
            break;
            case 19:
                if (!follower.isBusy()){
                    robot.intake.off();
                    robot.claw.setClose();
                    robot.vSlides.moveEncoderTo(robot.vSlides.autohigh1, 1f);
                    setPathState(20);
                }
                break;
            case 20:
                if(!follower.isBusy() && pathTimer.milliseconds()>200) {
                    robot.claw.setOpen();
                    robot.intakeTilt.setOut();
                    robot.intake.in();
                    setPathState(21);
                }
                break;
            case 21:
                if(pathTimer.milliseconds()>300){
                    robot.depoTilt.setTransfer();
                    robot.depoHslide.setTransfer();
                    follower.followPath(toBlock2, true);
                    setPathState(22);
                }
                break;
            case 22:
                if(pathTimer.milliseconds() > 150) {
                    vslideGoBottom = true;
                    in = false;
                    robot.hslides.moveEncoderTo(robot.hslides.PRESET3, 0.8f);
                    setPathState(100);
                }
                break;

                case 100: //TODO: test
                telems.addLine("CASE 100 - IN TEST CASE!!");
                break;
        }
    }


    // path setter
    public void setPathState(int state) {
        pathState = state;
        pathTimer.reset();
        autoPath();
    }

    @Override
    public void loop() {
        follower.update();
        robot.sensorF.update();
        robot.vSlides.update();
        autoPath();
        telemetry.addLine("TValue: " + follower.getCurrentTValue());
        telemetry.addLine("Case: " + pathState);
        telemetry.addLine("PathTimer: " + pathTimer);
        telemetry.addLine("Position: " + follower.getPose());
        telemetry.addLine("heading: " + follower.getTotalHeading());

        //functions
        if (hSlideGoBottom) {
            if (!hlimitswitch.getState() && hSlideGoBottom) {
                robot.latch.setInit();
                robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.hslides.hslides.setPower(-1f);
                RobotLog.ii(TAG_SL, "Going down");
            } else if (hlimitswitch.getState() && hSlideGoBottom) {
                in = true;
                robot.latch.setInit();
                robot.intakeTilt.setTransfer();
                robot.hslides.hslides.setPower(0);
                robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hSlideGoBottom = false;
            }
        }

        if (vslideGoBottom) { //vSlide bottom
            if (!vlimitswitch.getState() && vslideGoBottom) {
                robot.vSlides.vSlidesL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.vSlides.vSlidesR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.vSlides.vSlidesR.setPower(-0.6f);
                robot.vSlides.vSlidesL.setPower(-0.6f);
            } else if (vlimitswitch.getState() && vslideGoBottom) {
                robot.vSlides.vSlidesR.setPower(0);
                robot.vSlides.vSlidesL.setPower(0);
                robot.vSlides.vSlidesL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.vSlides.vSlidesR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                vslideGoBottom = false;
                robot.vSlides.vSlidesR.resetPosition();
                robot.vSlides.vSlidesL.resetPosition();
            }
        }
    }

    // initialize robot
    @Override
    public void init() {

        // Robot things init
        telems = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
        robot = new RobotMecanum(this, true, false);
        drive = new SampleMecanumDrive(hardwareMap);
        pathTimer = new ElapsedTime();

        //follower init
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        //Pose init
        buildPoses();
        buildPaths();

        //robot init
        robot.vSlides.vSlidesR.resetPosition();
        robot.vSlides.vSlidesL.resetPosition();
        hlimitswitch = hardwareMap.get(DigitalChannel.class, "hlimitswitch");
        vlimitswitch = hardwareMap.get(DigitalChannel.class, "vlimitswitch");
        robot.intakeTilt.setTransfer();
        robot.depoTilt.setTransfer();
        robot.claw.setMoreClose();
    }

    //loop de loop but initialized
   /* @Override
    public void init_loop() {

    }*/

    @Override
    public void start() {
        // starts auto paths
        setPathState(10);
        // safety net if auto doesn't start for some reason
        autoPath();
    }

    public static void waitFor(int milliseconds) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < milliseconds) {
        }
    }
}
