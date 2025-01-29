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
import overcharged.drive.SampleMecanumDrive;
import overcharged.pedroPathing.follower.Follower;
import overcharged.pedroPathing.localization.Pose;
import overcharged.pedroPathing.pathGeneration.BezierLine;
import overcharged.pedroPathing.pathGeneration.BezierPoint;
import overcharged.pedroPathing.pathGeneration.Path;
import overcharged.pedroPathing.pathGeneration.PathChain;
import overcharged.pedroPathing.pathGeneration.Point;
import overcharged.pedroPathing.util.Timer;

@Autonomous(name = "red specimen +4 faster faster FASTER", group = "1Autonomous")
public class autoRedSpecimenNEW extends OpMode {
    boolean vslideGoBottom = false;
    boolean hSlideGoBottom = false;
    boolean in = true;
    boolean stillBusy1 = false;
    boolean stillBusy2 = false;
    boolean stillBusy3 = false;
    boolean stillBusy4 = false;
    // Init
    private RobotMecanum robot;
    private DigitalChannel hlimitswitch;
    private DigitalChannel vlimitswitch;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SampleMecanumDrive drive;
    MultipleTelemetry telems;
    private ElapsedTime pathTimer;

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
    private Pose beforeSpecimen, atSpecimen, backUp, goPark, goForward, goRotate, bitForward, bitBack, toSample, secondScore, bitCloser, bitBitBack, thirdSample, getThirdSample, thirdScore, thirdScoreCloser, fourthScore, fourthScoreCloser, finalPark, grabFourthSample;
    private Pose startPose = new Pose(135, 66, Math.PI);

    private Path redPark, redPark2, slightMove, nextRotate, bitRotate, toSample2, grabSample, nextSample, getCloser, getGetBack, toSample3, grabSample3, scoreSample3, scoredSample3, scoreSample4, scoredSample4, endPark, grabSample4;

    private PathChain preload;

    private Follower follower;

    //TODO: Starting from here are the poses for the paths
    public void firstSpecimen(){
        //beforeBucket = new Pose(-10,-10,Math.PI/4);
        beforeSpecimen = new Pose(109,64,Math.PI);
        // atSpecimen = new Pose(117,70,0);
        goForward = new Pose(130,64, Math.PI);
        backUp = new Pose(124,98, Math.PI);
        goPark = new Pose(85,98, Math.PI);
        goRotate = new Pose(85,105, Math.PI);
        bitForward = new Pose(119,109, Math.PI);
        bitBack = new Pose(85,109, Math.PI);
        toSample = new Pose(85,116, Math.PI);
        secondScore = new Pose(119,116, Math.PI);
        bitCloser = new Pose(85,116, Math.PI);
        bitBitBack = new Pose(85,121, Math.PI);
        thirdSample = new Pose(119,121, Math.PI);
        getThirdSample = new Pose(131,110, Math.PI);
        thirdScore = new Pose(106,61, Math.PI);
        thirdScoreCloser = new Pose(133,102, Math.PI);
        fourthScore = new Pose(107,63, Math.PI);
        fourthScoreCloser = new Pose(107,64, Math.PI);
        grabFourthSample = new Pose(133,101, Math.PI);
        finalPark = new Pose(130,100, Math.PI);


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
                .addPath(new BezierLine(new Point(startPose),new Point(beforeSpecimen)))
                //.setConstantHeadingInterpolation(startPose.getHeading())
                .setLinearHeadingInterpolation(startPose.getHeading(), beforeSpecimen.getHeading())
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
        scoreSample4 = new Path(new BezierLine(new Point(thirdScoreCloser), new Point(fourthScore)));
        scoreSample4.setConstantHeadingInterpolation(Math.PI);
        grabSample4 = new Path(new BezierLine(new Point(fourthScore), new Point(grabFourthSample)));
        grabSample4.setConstantHeadingInterpolation(Math.PI);
        scoredSample4 = new Path(new BezierLine(new Point(grabFourthSample), new Point(fourthScoreCloser)));
        scoredSample4.setConstantHeadingInterpolation(Math.PI);
        endPark = new Path(new BezierLine(new Point(fourthScoreCloser), new Point(finalPark)));
        endPark.setConstantHeadingInterpolation(Math.PI);

    }




    // TODO: HERE IS WHERE THE MAIN PATH IS
    // Main pathing
    public void autoPath() {
        switch (pathState) {
            // Auto Body
            //
            case 10: // scores initial specimen
                pathTimer.reset();
                robot.claw.setClose();
                waitFor(300);
                follower.followPath(preload);
                robot.vSlides.moveEncoderTo(robot.vSlides.mid, 1);
                robot.depoHslide.setOut();
                robot.clawBigTilt.setOut();
                robot.clawSmallTilt.setFlat();
                setPathState(13);
                break;
            case 13: // scores initial specimen
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(beforeSpecimen)), Math.toRadians(180));
                    waitFor(200);
                    robot.claw.setOpen();
                    waitFor(200);
                    robot.depoHslide.setInit();
                    waitFor(200);
                    follower.followPath(redPark);
                    setPathState(15);
                    waitFor(300);
                    stillBusy1 = true;
                }
                break;
            case 15:
                if(stillBusy1 && follower.isBusy()) {
                    stillBusy1 = false;
                    robot.depoWrist.setIn();
                    robot.claw.setOpen();
                    robot.clawSmallTilt.setWall();
                    robot.clawBigTilt.setWall();
                    robot.intakeTilt.setOut();
                    robot.intake.off();
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid-30, 1f);
                    vslideGoBottom = true;
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(backUp)), Math.toRadians(180));
                    follower.followPath(redPark2);
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(goPark)), Math.toRadians(180));
                    follower.followPath(nextRotate);
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    //follower.holdPoint(new BezierPoint(new Point(goRotate)), Math.toRadians(180));
                    follower.followPath(bitRotate);
                    setPathState(19);
                }
                break;
            case 19:
                if(!follower.isBusy()) {
                    //follower.holdPoint(new BezierPoint(new Point(goRotate)), Math.toRadians(180));
                    follower.followPath(toSample2);
                    setPathState(20);
                }
                break;
            case 20:
                if(!follower.isBusy()) {
                    //follower.holdPoint(new BezierPoint(new Point(goRotate)), Math.toRadians(180));
                    follower.followPath(grabSample);
                    setPathState(21);
                }
                break;
            case 21:
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(toSample)), Math.toRadians(180));
                    follower.followPath(nextSample);
                    setPathState(25);
                }
                break;
           /* case 22:
                if(!follower.isBusy()) {
                    //follower.holdPoint(new BezierPoint(new Point(goRotate)), Math.toRadians(180));
                    follower.followPath(getCloser);
                    setPathState(23);
                }
                break;
            case 23:
                if(!follower.isBusy()) {
                    //follower.holdPoint(new BezierPoint(new Point(goRotate)), Math.toRadians(180));
                    follower.followPath(getGetBack);
                    setPathState(24);
                }
                break;
            case 24:
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(thirdSample)), Math.toRadians(180));
                    follower.followPath(toSample3);
                    setPathState(25);
                }
                break;*/
            case 25:
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(secondScore)), Math.toRadians(180));
                    follower.followPath(grabSample3);
                    setPathState(26);
                }
                break;
            case 26:
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(getThirdSample)), Math.toRadians(180));
                    robot.claw.setClose();
                    robot.vSlides.moveEncoderTo(80, 1f);
                    setPathState(27);
                }
                break;
            case 27:
                if(!follower.isBusy()) {
                    follower.followPath(scoreSample3);
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid, 1f);
                    robot.claw.setClose();
                    robot.clawBigTilt.setOut();
                    robot.depoHslide.setOut();
                    robot.clawSmallTilt.setFlat();
                    setPathState(28);
                }
                break;
            case 28:
                if(!follower.isBusy()){
                    follower.holdPoint(new BezierPoint(new Point(thirdScore)), Math.toRadians(179));
                    robot.claw.setOpen();
                    //follower.holdPoint(new BezierPoint(new Point(getThirdSample)), Math.toRadians(180));
                    robot.depoHslide.setInit();
                    waitFor(200);
                    follower.followPath(scoredSample3);
                    waitFor(200);
                    robot.intakeTilt.setOut();
                    robot.depoWrist.setIn();
                    robot.claw.setOpen();
                    robot.clawBigTilt.setWall();
                    robot.clawSmallTilt.setWall();
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid-30, 1f);
                    vslideGoBottom = true;
                    setPathState(29);
                }
                break;
            case 29:
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(thirdScoreCloser)), Math.toRadians(180));
                    robot.claw.setClose();
                    robot.vSlides.moveEncoderTo(80, 1f);
                    setPathState(30);
                }
                break;
            case 30:
                if(!follower.isBusy()) {
                    follower.followPath(scoreSample4);
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid, 1f);
                    robot.claw.setClose();
                    robot.clawBigTilt.setOut();
                    robot.depoHslide.setOut();
                    robot.clawSmallTilt.setFlat();
                    setPathState(31);
                }
                break;
            case 31:
                if(!follower.isBusy()){
                    follower.holdPoint(new BezierPoint(new Point(fourthScore)), Math.toRadians(180));
                    robot.claw.setOpen();
                    //follower.holdPoint(new BezierPoint(new Point(getThirdSample)), Math.toRadians(180));
                    waitFor(200);
                    robot.depoHslide.setInit();
                    follower.followPath(grabSample4);
                    waitFor(200);
                    robot.intakeTilt.setOut();
                    robot.depoWrist.setIn();
                    robot.claw.setOpen();
                    robot.clawBigTilt.setWall();
                    robot.clawSmallTilt.setWall();
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid-30, 1f);
                    vslideGoBottom = true;
                    setPathState(32);
                }
                break;
            case 32:
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(grabFourthSample)), Math.toRadians(180));
                    robot.claw.setClose();
                    robot.vSlides.moveEncoderTo(80, 1f);
                    setPathState(33);
                }
                break;
            case 33:
                if(!follower.isBusy()) {
                    follower.followPath(scoredSample4);
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid, 1f);
                    robot.claw.setClose();
                    robot.clawBigTilt.setOut();
                    robot.depoHslide.setOut();
                    robot.clawSmallTilt.setFlat();
                    setPathState(34);
                }
                break;
            case 34:
                if(!follower.isBusy()){
                    follower.holdPoint(new BezierPoint(new Point(fourthScoreCloser)), Math.toRadians(180));
                    robot.claw.setOpen();
                    //follower.holdPoint(new BezierPoint(new Point(getThirdSample)), Math.toRadians(180));
                    waitFor(200);
                    robot.depoHslide.setInit();
                    follower.followPath(endPark);
                    waitFor(200);
                    robot.intakeTilt.setOut();
                    robot.depoWrist.setIn();
                    robot.claw.setOpen();
                    robot.clawBigTilt.setTransfer();
                    robot.clawSmallTilt.setTransfer();
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid-30, 1f);
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
        pathTimer.reset();
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

        if (!vlimitswitch.getState() && vslideGoBottom) {
            robot.vSlides.vSlidesL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.vSlides.vSlidesR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.vSlides.vSlidesR.setPower(-0.8f);
            robot.vSlides.vSlidesL.setPower(-0.8f);
        } else if (vlimitswitch.getState() && vslideGoBottom) {
            robot.vSlides.vSlidesR.setPower(0);
            robot.vSlides.vSlidesL.setPower(0);
            robot.vSlides.vSlidesL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.vSlides.vSlidesR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            vslideGoBottom = false;
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

        robot.vSlides.vSlidesR.resetPosition();
        robot.vSlides.vSlidesL.resetPosition();

        //follower init
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);


        //Pose init
        firstSpecimen();
        buildPaths();

        //robot init
        robot.intakeTilt.setOut();
        robot.clawBigTilt.setOut();
        robot.clawSmallTilt.setFlat();
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

