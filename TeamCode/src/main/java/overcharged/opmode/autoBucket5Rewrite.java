package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import overcharged.pedroPathing.constants.FConstants;
import overcharged.pedroPathing.constants.LConstants;

@Autonomous(name = "new bucket", group = "0Autonomous")
public class autoBucket5Rewrite extends OpMode{

    // Init
    private RobotMecanum robot;
    private DigitalChannel hlimitswitch;
    private DigitalChannel vlimitswitch;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SampleMecanumDrive drive;
    MultipleTelemetry telems;
    private ElapsedTime pathTimer;

    ElapsedTime temp;

    private int pathState;

    private Follower follower;

    boolean vslideGoBottom = false;
    boolean hSlideGoBottom = false;
    boolean in = true;
    int turnExtra = 0;

    boolean runOnce;

    long tempTime;

    private int freeBlocks = 1;

    //TODO: poses
    private Pose startPose = new Pose(137, 31, Math.toRadians(90));
    private Pose bucketScore;

    private Path poop;

    private PathChain initScore;

    public void buildPoses() {
        bucketScore = new Pose(126.5, 12, Math.toRadians(155));
    }

    //TODO: here are where the paths are defined
    public void buildPaths() {
        initScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose),new Point(bucketScore)))
                .setLinearHeadingInterpolation(startPose.getHeading(), bucketScore.getHeading())
                .build();
    }


    // TODO: HERE IS WHERE THE MAIN PATH IS
    // Main pathing
    public void autoPath() {
        switch (pathState) {
            case 10: // start
                robot.intakeTilt.setTransfer();
                robot.vSlides.moveEncoderTo(robot.vSlides.high1, 1f);
                follower.followPath(initScore, true);
                setPathState(102);
                break;
            case 102:
                if(pathTimer.milliseconds()>250){
                    robot.vSlides.setUseSquID(true, vSlides.high1, 1f);
                    setPathState(11);
                }
                break;
            case 11:
                if(pathTimer.milliseconds()>250){
                    robot.latch.setOut();
                    robot.hslides.moveEncoderTo(robot.hslides.SMALL, 0.7f);
                    robot.depoTilt.setOut();
                    robot.depoHslide.setOut();
                    setPathState(13);
                }
                break;
            case 13:
                if ((follower.getPose().getX() > (bucketScore.getX() - 1.5) && follower.getPose().getY() < (bucketScore.getY() + 1.5) && Math.abs(robot.vSlides.vSlidesL.getCurrentPosition()-660) < 16)|| pathTimer.milliseconds()>1300) {
                    robot.claw.setOpen();
                    robot.intakeTilt.setOut();
                    robot.intake.in();
                    setPathState(15);
                }
                break;
            case 15:
                if(pathTimer.milliseconds()>250){
                    robot.depoTilt.setTransfer();
                    robot.depoHslide.setTransfer();
                    follower.turnTo(Math.toDegrees(155+turnExtra*10));
                    setPathState(16);
                }
                break;
            case 16:
                if(pathTimer.milliseconds() > 300) {
                    robot.vSlides.setUseSquID(false, 0);
                    vslideGoBottom = true;
                    if (turnExtra == 1) {
                        robot.hslides.moveEncoderTo(robot.hslides.PRESET3, 1f);
                    } else if (turnExtra == 2){
                        robot.hslides.moveEncoderTo(robot.hslides.PRESET2, 1f);
                    } else if (turnExtra == 3){
                        robot.hslides.moveEncoderTo(robot.hslides.PRESET1, 1f);
                    }
                    setPathState(17);
                    runOnce = true;
                }
                break;
            case 17:
                if(robot.sensorF.getColor() == colorSensor.Color.YELLOW){
                    setPathState(171);
                }
                else if(robot.sensorF.getColor() == colorSensor.Color.NONE && pathTimer.milliseconds()<1900 && robot.hslides.hslides.getCurrentPosition() < 590) {
                    robot.hslides.moveEncoderTo(robot.hslides.hslides.getCurrentPosition() + 90, 1f);
                    }
                break;
            case 171:
                if (runOnce){
                    runOnce = false;
                    robot.intakeTilt.setTransfer();
                    robot.intake.in();
                }
                if(pathTimer.milliseconds() > 300) {
                    hSlideGoBottom = true;
                    setPathState(1711);
                    tempTime = 0;

                }
                break;
            case 1711:
                if(in){
                    robot.claw.setClose();
                    if (turnExtra<4) {
                        turnExtra += 1;
                        setPathState(102);
                    }
                }
                break;


            case 100: //TODO: test
                telems.addLine("CASE 100 - IN TEST CASE!!");
                break;
        }
    }


    // path setter
    public void setPathState(int state){
        pathState = state;
        pathTimer.reset();
        autoPath();
    }

    //loop de loop
    @Override
    public void loop() {
        temp.reset();
        follower.update();
        robot.sensorF.update();
        autoPath();
        telemetry.addLine("TValue: " + follower.getCurrentTValue());
        telemetry.addLine("Case: " + pathState);
        telemetry.addLine("Position: " + follower.getPose());
        telemetry.addLine("heading: " + follower.getTotalHeading());
        telemetry.addData("lag: ", temp);
        telemetry.addLine("color: "+robot.sensorF.getColor());

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
        temp = new ElapsedTime();



        //follower init
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        //Pose init
        buildPoses();
        buildPaths();

        //robot init
        hlimitswitch = hardwareMap.get(DigitalChannel.class, "hlimitswitch");
        vlimitswitch = hardwareMap.get(DigitalChannel.class, "vlimitswitch");
        robot.intakeTilt.setTransfer();
        robot.depoTilt.setTransfer();
        robot.claw.setClose();
    }

    //loop de loop but initialized
    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        // starts auto paths
        setPathState(10);

        // safety net if auto doesn't start for some reason
        autoPath();
    }
}
