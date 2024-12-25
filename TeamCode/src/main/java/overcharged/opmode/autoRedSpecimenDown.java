package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.RobotMecanum;
import overcharged.components.colorSensor;
import overcharged.drive.SampleMecanumDrive;
import overcharged.linear.util.SelectLinear;

import overcharged.pedroPathing.follower.Follower;
import overcharged.pedroPathing.localization.Pose;
import overcharged.pedroPathing.pathGeneration.BezierCurve;
import overcharged.pedroPathing.pathGeneration.BezierLine;
import overcharged.pedroPathing.pathGeneration.BezierPoint;
import overcharged.pedroPathing.pathGeneration.MathFunctions;
import overcharged.pedroPathing.pathGeneration.Path;
import overcharged.pedroPathing.pathGeneration.PathChain;
import overcharged.pedroPathing.pathGeneration.Point;
import overcharged.pedroPathing.util.Timer;

// Main Class
@Autonomous(name = "red specimen arm down", group = "Autonomous")
public class autoRedSpecimenDown extends OpMode{

    //stuff
    boolean vslideGoBottom = false;
    boolean hSlideGoBottom = false;

    // Init
    private RobotMecanum robot;
    private DigitalChannel hlimitswitch;
    private DigitalChannel vlimitswitch;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SampleMecanumDrive drive;
    MultipleTelemetry telems;
    private Timer pathTimer, opmodeTimer, scanTimer, distanceSensorUpdateTimer, distanceSensorDecimationTimer;

    // Other init
    private int pathState;

    // LOCATIONS
    // GUIDE:
    // (0,0) is the corner. (144, 144) is the opposite.

    // TODO: Here are all the Sample Poses
    // Blue side Blue Element Poses
    //
    private Pose blueBlueLeftSample = new Pose(2.5+9.75+10.5, 4*24+1.5);
    private Pose blueBlueMidSample = new Pose(2.5+9.75, 4*24+1.5);
    private Pose blueBlueRightSample = new Pose(2.5, 4*24+1.5);
    // Blue side Neutral Element Poses
    //
    private Pose blueNeutLeftSample = new Pose(144-2.5, 4*24+1.5);
    private Pose blueNeutMidSample = new Pose(144-2.5-9.75, 4*24+1.5);
    private Pose blueNeutRightSample = new Pose(144-2.5-9.75-10.5, 4*24+1.5);
    // Red Side Red Element Poses
    //
    private Pose redRedLeftSample = new Pose(144-2.5-9.75-10.5, 2*24-2.5);
    private Pose redRedMidSample = new Pose(144-2.5-9.75, 2*24-2.5);
    private Pose redRedRightSample = new Pose(144-2.5, 2*24-2.5);
    // Red Side Neutral Element Poses
    //
    private Pose redNeutLeftSample = new Pose(2.5, 2*24-2.5);
    private Pose redNeutMidSample = new Pose(2.5+9.75, 2*24-2.5);
    private Pose redNeutRightSample = new Pose(2.5+9.75+10.5, 2*24-2.5);

    // TODO: Here are the Basket and Observation Zone positions
    // Blue side Left Basket Pose
    private Pose blueLeftBasket = new Pose();
    // Blue side Right Basket Pose
    private Pose blueRightBasket = new Pose();
    // Red side Left Basket Pose
    private Pose redLeftBasket = new Pose();
    // Red side Right Basket Pose
    private Pose redRightBasket = new Pose();

    // OTHER POSES
    private Pose beforeSpecimen, atSpecimen, backUp, goPark, goForward, backward;
    private Pose startPose = new Pose(135, 64, 0);

    private Path redPark, redPark2, slightMove,goBack;

    private PathChain preload;

    private Follower follower;

    //TODO: Starting from here are the poses for the paths
    public void firstSpecimen(){
        //beforeBucket = new Pose(-10,-10,Math.PI/4);
        beforeSpecimen = new Pose(116,64,0);
        // atSpecimen = new Pose(117,70,0);
        goForward = new Pose(133,64,0);
        backUp = new Pose(120,64,0);
        goPark = new Pose(132,110,0);
        backward = new Pose(134,64, Math.PI);

    }


    //TODO: here are where the paths are defined
    public void buildPaths() {
        /*
        firstScore = new Path(new BezierLine(new Point(startPose),new Point(beforeBucket)));
        firstScore.setConstantHeadingInterpolation(Math.PI/2);

        inchBucket = new Path(new BezierLine(new Point(beforeBucket), new Point(ready2Score)));
        inchBucket.setConstantHeadingInterpolation(3*Math.PI/4);

         */

        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(goForward),new Point(beforeSpecimen)))
                //.setConstantHeadingInterpolation(startPose.getHeading())
                .setLinearHeadingInterpolation(goForward.getHeading(), beforeSpecimen.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        // slightMove = new Path(new BezierLine(new Point(beforeSpecimen), new Point(atSpecimen)));
        //slightMove.setConstantHeadingInterpolation(0);

        slightMove = new Path(new BezierLine(new Point(startPose), new Point(goForward)));
        slightMove.setConstantHeadingInterpolation(0);
        redPark = new Path(new BezierLine(new Point(beforeSpecimen), new Point(backUp)));
        redPark.setConstantHeadingInterpolation(0);
        redPark2 = new Path(new BezierLine(new Point(backUp), new Point(goPark)));
        redPark2.setConstantHeadingInterpolation(0);
        goBack = new Path(new BezierLine(new Point(backUp), new Point(backward)));
        goBack.setConstantHeadingInterpolation(Math.PI);

    }




    // TODO: HERE IS WHERE THE MAIN PATH IS
    // Main pathing
    public void autoPath() {
        switch (pathState) {
            // Auto Body
            //
            case 10: // scores initial specimen
                pathTimer.resetTimer();
                follower.followPath(slightMove);
                slightMove.setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(0));
                setPathState(11);
                break;
            case 11: // scores initial specimen
                if(!follower.isBusy()) {
                    waitFor(100);
                    robot.intakeTilt.setOut();
                    robot.clawBigTilt.setWall();
                    robot.clawSmallTilt.setWall();
                    robot.depoWrist.setIn();
                    waitFor(1000);
                    setPathState(12);
                }
                break;
            case 12: // scores initial specimen
                if(!follower.isBusy()) {
                    robot.claw.setClose();
                    waitFor(1000);
                    setPathState(13);
                }
                break;
            case 13: // scores initial specimen
                if(!follower.isBusy()) {
                    follower.followPath(preload);
                    waitFor(200);
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid+50, 1f);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    waitFor(700);
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid-270, -1f);
                    waitFor(1000);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    waitFor(100);
                    robot.claw.setOpen();
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    waitFor(500);
                    follower.followPath(redPark);
                    redPark.setLinearHeadingInterpolation(backUp.getHeading(), Math.toRadians(0));
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    robot.intakeTilt.setOut();
                    robot.depoWrist.setIn();
                    waitFor(100);
                    robot.claw.setOpen();
                    robot.clawBigTilt.setTransfer();
                    robot.clawSmallTilt.setTransfer();
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid-200, 1f);
                    waitFor(500);
                    vslideGoBottom = true;
                    waitFor(500);
                    robot.intakeTilt.setTransfer();
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    waitFor(100);
                    follower.followPath(goBack);
                    goBack.setLinearHeadingInterpolation(backward.getHeading(), Math.toRadians(180));
                    setPathState(100);
                }
                break;

            case 100: // EMPTY TEST CASE
                //follower.holdPoint(new BezierPoint(firstScore.getLastControlPoint()), Math.toRadians(-90));
                telems.addLine("CASE 100 - IN TEST CASE!!");
                break;

        }
    }


    // path setter
    public void setPathState(int state){
        pathState = state;
        pathTimer.resetTimer();
        autoPath();
    }

    // Distance Sensor Checker
    public void startDistanceSensorDisconnectDetection(int state) {
    }

    //loop de loop
    @Override
    public void loop() {
        follower.update();
        autoPath();
        telemetry.addLine("TValue: "+follower.getCurrentTValue());
        telemetry.addLine("Path: " + pathState);
        telemetry.addLine("vLimit" + vlimitswitch.getState());
        telemetry.addLine("hLimit" + hlimitswitch.getState());

        //functions
        if (!hlimitswitch.getState() && hSlideGoBottom) {
            robot.latch.setInit();
            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.hslides.hslides.setPower(-1);
            RobotLog.ii(TAG_SL, "Going down");
        } else if (hlimitswitch.getState() && hSlideGoBottom) {
            robot.latch.setInit();
            // robot.intakeTilt.setTransfer();
            robot.hslides.hslides.setPower(0);
            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hSlideGoBottom = false;
            RobotLog.ii(TAG_SL, "Force stopped");
        }

        if (vlimitswitch.getState() && vslideGoBottom) {
            robot.vSlides.vSlidesL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.vSlides.vSlidesR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.vSlides.vSlidesR.setPower(-1);
            robot.vSlides.vSlidesL.setPower(-1);
            RobotLog.ii(TAG_SL, "Going down");
        } else if (!vlimitswitch.getState() && vslideGoBottom) {
            //robot.hslides.forceStop();
            robot.vSlides.vSlidesR.setPower(0);
            robot.vSlides.vSlidesL.setPower(0);
            robot.vSlides.vSlidesL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.vSlides.vSlidesR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            vslideGoBottom = false;
            RobotLog.ii(TAG_SL, "Force stopped");
        }
    }

    // initialize robot
    @Override
    public void init() {

        // Robot things init
        telems = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
        robot = new RobotMecanum(this, true, false);
        drive = new SampleMecanumDrive(hardwareMap);
        pathTimer = new Timer();



        //follower init
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);



        //Pose init
        firstSpecimen();
        buildPaths();

        //robot init
        robot.claw.setClose();
        hlimitswitch = hardwareMap.get(DigitalChannel.class, "hlimitswitch");
        vlimitswitch = hardwareMap.get(DigitalChannel.class, "vlimitswitch");;

    }

    //loop de loop but initialized
    /*@Override
    public void init_loop() {

    }*/

    @Override
    public void start() {
        // starts auto paths
        setPathState(10);

        // safety net if auto doesn't start for some reason
        autoPath();
    }

    public static void waitFor(int milliseconds) { //Waitor Function
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < milliseconds) {
            // loop
        }
    }
}

