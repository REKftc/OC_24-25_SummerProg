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
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

@Autonomous(name = "red specimen +3 faster", group = "Autonomous")
public class autoRedSpecimenFast3 extends OpMode {
    boolean vslideGoBottom = false;
    boolean hSlideGoBottom = false;
    boolean in = true;
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
    private Pose beforeSpecimen, atSpecimen, backUp, goPark, goForward, goRotate, bitForward, bitBack, toSample, secondScore, bitCloser, bitBitBack, thirdSample, getThirdSample, thirdScore, thirdScoreCloser, fourthScore, fourthScoreCloser;
    private Pose startPose = new Pose(135, 66, Math.PI);

    private Path redPark, redPark2, slightMove, nextRotate, bitRotate, toSample2, grabSample, nextSample, getCloser, getGetBack, toSample3, grabSample3, scoreSample3, scoredSample3, scoreSample4, scoredSample4;

    private PathChain preload;

    private Follower follower;

    //TODO: Starting from here are the poses for the paths
    public void firstSpecimen(){
        //beforeBucket = new Pose(-10,-10,Math.PI/4);
        beforeSpecimen = new Pose(114,68,Math.PI);
        // atSpecimen = new Pose(117,70,0);
        goForward = new Pose(130,68, Math.PI);
        backUp = new Pose(119,68, Math.PI);
        goPark = new Pose(121,104, Math.PI);
        goRotate = new Pose(125,118, Math.PI);
        bitForward = new Pose(123,116.5, 3*Math.PI/4);
        bitBack = new Pose(123,117.5, Math.PI);
        toSample = new Pose(132,116, Math.PI);
        secondScore = new Pose(131,67, Math.PI);
        bitCloser = new Pose(111,61, Math.PI);
        bitBitBack = new Pose(122,59, Math.PI);
        thirdSample = new Pose(122,105, Math.PI);
        getThirdSample = new Pose(131,110, Math.PI);
        thirdScore = new Pose(131,62, Math.PI);
        thirdScoreCloser = new Pose(110,56, Math.PI);
        fourthScore = new Pose(129,70, Math.PI);
        fourthScoreCloser = new Pose(114,64, Math.PI);


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
        slightMove.setConstantHeadingInterpolation(Math.PI);
        redPark = new Path(new BezierLine(new Point(beforeSpecimen), new Point(backUp)));
        redPark.setConstantHeadingInterpolation(Math.PI);
        redPark2 = new Path(new BezierLine(new Point(backUp), new Point(goPark)));
        redPark2.setConstantHeadingInterpolation(Math.PI);
        nextRotate = new Path(new BezierLine(new Point(goPark), new Point(goRotate)));
        nextRotate.setConstantHeadingInterpolation(Math.PI);
        bitRotate = new Path(new BezierLine(new Point(goRotate), new Point(bitForward)));
        bitRotate.setConstantHeadingInterpolation(Math.PI);
        toSample2 = new Path(new BezierLine(new Point(bitForward), new Point(bitBack)));
        toSample2.setConstantHeadingInterpolation(Math.PI);
        grabSample = new Path(new BezierLine(new Point(bitBack), new Point(toSample)));
        grabSample.setConstantHeadingInterpolation(Math.PI);
        nextSample = new Path(new BezierLine(new Point(toSample), new Point(secondScore)));
        nextSample.setConstantHeadingInterpolation(Math.PI);
        getCloser = new Path(new BezierLine(new Point(secondScore), new Point(bitCloser)));
        getCloser.setConstantHeadingInterpolation(Math.PI);
        getGetBack = new Path(new BezierLine(new Point(bitCloser), new Point(bitBitBack)));
        getGetBack.setConstantHeadingInterpolation(Math.PI);
        toSample3 = new Path(new BezierLine(new Point(bitBitBack), new Point(thirdSample)));
        toSample3.setConstantHeadingInterpolation(Math.PI);
        grabSample3 = new Path(new BezierLine(new Point(thirdSample), new Point(getThirdSample)));
        grabSample3.setConstantHeadingInterpolation(Math.PI);
        scoreSample3 = new Path(new BezierLine(new Point(getThirdSample), new Point(thirdScore)));
        scoreSample3.setConstantHeadingInterpolation(Math.PI);
        scoredSample3 = new Path(new BezierLine(new Point(thirdScore), new Point(thirdScoreCloser)));
        scoredSample3.setConstantHeadingInterpolation(Math.PI);
        scoreSample4 = new Path(new BezierLine(new Point(getThirdSample), new Point(fourthScore)));
        scoreSample4.setConstantHeadingInterpolation(Math.PI);
        scoredSample4 = new Path(new BezierLine(new Point(fourthScore), new Point(fourthScoreCloser)));
        scoredSample4.setConstantHeadingInterpolation(Math.PI);

    }




    // TODO: HERE IS WHERE THE MAIN PATH IS
    // Main pathing
    public void autoPath() {
        switch (pathState) {
            // Auto Body
            //
            case 10: // scores initial specimen
                pathTimer.resetTimer();
                robot.claw.setClose();
                waitFor(300);
                robot.vSlides.moveEncoderTo(robot.vSlides.mid+95, 1.2f);
                follower.followPath(preload);
                robot.clawBigTilt.setOut();
                robot.depoHslide.setOut();
                robot.clawSmallTilt.setFlat();
                setPathState(13);
                break;
            case 13: // scores initial specimen
                if(!follower.isBusy()) {
                    waitFor(700);
                    robot.claw.setOpen();
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    waitFor(100);
                    follower.followPath(redPark);
                    redPark.setLinearHeadingInterpolation(backUp.getHeading(), Math.toRadians(180));
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    robot.depoHslide.setInit();
                    robot.intakeTilt.setOut();
                    robot.depoWrist.setIn();
                    robot.claw.setOpen();
                    robot.clawBigTilt.setTransfer();
                    robot.clawSmallTilt.setTransfer();
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid-50, 1f);
                    vslideGoBottom = true;
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    waitFor(100);
                    follower.followPath(redPark2);
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(goPark)), Math.toRadians(180));
                    robot.intakeTilt.setOut();
                    robot.intake.in();
                    robot.latch.setOut();
                    in = false;
                    robot.hslides.moveEncoderTo(450, 0.8f);
                    setPathState(18);
                }
                break;
            case 18:
                if(robot.sensorF.getColor() == colorSensor.Color.RED){
                    robot.intakeTilt.setTransfer();
                    hSlideGoBottom = true;
                    waitFor(250);
                    robot.intake.off();
                    setPathState(19);
                }
                break;
            case 19:
                // if(hlimitswitch.getState()){
                if(in) {
                    robot.intakeTilt.setTransfer();
                    waitFor(600);
                    robot.claw.setClose();
                    waitFor(200);
                    setPathState(20);
                }
                break;
            case 20: // scores initial specimen
                if(!follower.isBusy()) {
                    robot.intakeTilt.setOut();
                    waitFor(200);
                    robot.clawBigTilt.setWall();
                    robot.clawSmallTilt.setWall();
                    waitFor(700);
                    robot.claw.setOpen();
                    waitFor(1000);
                    setPathState(31);
                }
                break;
            case 31: // scores initial specimen
                if(!follower.isBusy()) {
                    follower.followPath(grabSample);
                    setPathState(32);
                }
                break;
            case 32: // scores initial specimen
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(toSample)), Math.toRadians(180));
                    waitFor(500);
                    robot.claw.setClose();
                    setPathState(33);
                }
                break;
            case 33: // scores initial specimen
                if(!follower.isBusy()) {
                    follower.followPath(getCloser);
                    robot.vSlides.moveEncoderTo(80, 1.2f);
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid+90, 1.2f);
                    robot.claw.setClose();
                    robot.clawBigTilt.setOut();
                    robot.depoHslide.setOut();
                    robot.clawSmallTilt.setFlat();
                    setPathState(36);
                }
                break;
            case 36: // scores initial specimen
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(bitCloser)), Math.toRadians(180));
                    robot.claw.setOpen();
                    waitFor(150);
                    follower.followPath(getGetBack);
                    setPathState(37);
                }
                break;
            case 37: // scores initial specimen
                if(!follower.isBusy()) {
                    robot.depoHslide.setInit();
                    robot.intakeTilt.setOut();
                    robot.depoWrist.setIn();
                    robot.claw.setOpen();
                    robot.clawBigTilt.setWall();
                    robot.clawSmallTilt.setWall();
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid-50, 1f);
                    vslideGoBottom = true;
                    setPathState(38);
                }
                break;
            case 38: // scores initial specimen
                if(!follower.isBusy()) {
                    follower.followPath(toSample3);
                    setPathState(39);
                }
                break;
            case 39: // scores initial specimen
                if(!follower.isBusy()) {
                    follower.followPath(grabSample3);
                    setPathState(40);
                }
                break;
            case 40: // scores initial specimen
                if(!follower.isBusy()) {
                    waitFor(600);
                    robot.claw.setClose();
                    setPathState(41);
                }
                break;
            case 41: // scores initial specimen
                if(!follower.isBusy()) {
                    follower.followPath(scoredSample3);
                    robot.vSlides.moveEncoderTo(80, 1.2f);
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid+90, 1.2f);
                    robot.claw.setClose();
                    robot.clawBigTilt.setOut();
                    robot.depoHslide.setOut();
                    robot.clawSmallTilt.setFlat();
                    setPathState(43);
                }
                break;
            case 43: // scores initial specimen
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(bitCloser)), Math.toRadians(180));
                    robot.claw.setOpen();
                    waitFor(150);
                    follower.followPath(getGetBack);
                    setPathState(44);
                }
                break;
            case 44: // scores initial specimen
                if(!follower.isBusy()) {
                    robot.depoHslide.setInit();
                    robot.intakeTilt.setOut();
                    robot.depoWrist.setIn();
                    robot.claw.setOpen();
                    robot.clawBigTilt.setTransfer();
                    robot.clawSmallTilt.setTransfer();
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid-50, 1f);
                    vslideGoBottom = true;
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
            // in = true;
            robot.latch.setInit();
            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.hslides.hslides.setPower(-1.3f);
            RobotLog.ii(TAG_SL, "Going down");
        } else if (hlimitswitch.getState() && hSlideGoBottom) {
            robot.latch.setInit();
            // robot.intakeTilt.setTransfer();
            robot.hslides.hslides.setPower(0);
            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hSlideGoBottom = false;
            in = true;
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
        robot.intakeTilt.setOut();
        robot.clawBigTilt.setWall();
        robot.clawSmallTilt.setWall();
        robot.depoWrist.setIn();
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

