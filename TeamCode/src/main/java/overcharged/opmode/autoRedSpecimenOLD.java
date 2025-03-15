package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.RobotMecanum;
import overcharged.components.colorSensor;
import overcharged.drive.SampleMecanumDrive;
import overcharged.pedroPathing.constants.FConstants;
import overcharged.pedroPathing.constants.LConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

@Autonomous(name = "red specimen 1+4 old", group = "1Autonomous")
public class autoRedSpecimenOLD extends OpMode {
    boolean vslideGoBottom = false;
    boolean hSlideGoBottom = false;
    boolean in = true;
    boolean stillBusy1 = false;
    boolean stillBusy2 = false;
    boolean stillBusy3 = false;
    boolean stillBusy4 = false;
    // Init
    private RobotMecanum robot;
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
    private Pose beforeSpecimen, atSpecimen, backUp, goPark, goForward, goRotate, bitForward, bitBack, toSample, secondScore, bitCloser, bitBitBack, thirdSample, getThirdSample, thirdScore, thirdScoreCloser, fourthScore, fourthScoreCloser, finalPark, grabFourthSample, pushFirstSample, pushThirdSample, pushThirdSampleSide, pushThirdSampleBack, grabFifthSample, scoreFifthSample;
    private Pose startPose = new Pose(135, 66, Math.PI);

    private Path redPark, redPark2, slightMove, nextRotate, bitRotate, toSample2, grabSample, nextSample, getCloser, getGetBack, toSample3, grabSample3, scoreSample3, scoredSample3, scoreSample4, scoredSample4, endPark, grabSample4, firstSamplePush, goingToThirdSample, sideToThirdSample, pushingThirdSample, getFifthSample, scoredFifthSample;

    private PathChain preload;

    private Follower follower;

    int step = 0;

    //TODO: Starting from here are the poses for the paths
    public void firstSpecimen(){
        //beforeBucket = new Pose(-10,-10,Math.PI/4);
        beforeSpecimen = new Pose(112,64,Math.PI);
        // atSpecimen = new Pose(117,70,0);
        goForward = new Pose(130,64, Math.PI);
        backUp = new Pose(120,64, Math.PI);
        pushFirstSample = new Pose(120,95, Math.PI);
        goPark = new Pose(89,98, Math.PI);
        goRotate = new Pose(89,105, Math.PI);
        bitForward = new Pose(118,109, Math.PI);
        bitBack = new Pose(89,109, Math.PI);
        toSample = new Pose(89,115, Math.PI);
        secondScore = new Pose(118,115, Math.PI);
        pushThirdSample = new Pose(89,120, Math.PI);
        pushThirdSampleSide = new Pose(89,123, Math.PI);
        pushThirdSampleBack = new Pose(115,123, Math.PI); //need to tune this, needs more back prob
        bitCloser = new Pose(89,116, Math.PI);
        bitBitBack = new Pose(89,121, Math.PI);
        thirdSample = new Pose(119,121, Math.PI);
        getThirdSample = new Pose(126,108, Math.PI);
        thirdScore = new Pose(106,70, Math.PI);
        thirdScoreCloser = new Pose(128,95, Math.PI);
        fourthScore = new Pose(106,68, Math.PI);
        fourthScoreCloser = new Pose(106,68, Math.PI);
        grabFourthSample = new Pose(128,95, Math.PI); //maybe this too 134 maybe
        grabFifthSample = new Pose(128,95, Math.PI);
        scoreFifthSample = new Pose(106,68, Math.PI);
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
        slightMove.setPathEndTimeoutConstraint(0);
        redPark = new Path(new BezierLine(new Point(beforeSpecimen), new Point(backUp)));
        redPark.setConstantHeadingInterpolation(Math.PI);
        redPark.setPathEndTimeoutConstraint(0);
        firstSamplePush = new Path(new BezierLine(new Point(backUp), new Point(pushFirstSample)));
        firstSamplePush.setConstantHeadingInterpolation(Math.PI);
        firstSamplePush.setZeroPowerAccelerationMultiplier(3.75);
        firstSamplePush.setPathEndTimeoutConstraint(0);
        redPark2 = new Path(new BezierLine(new Point(pushFirstSample), new Point(goPark)));
        redPark2.setConstantHeadingInterpolation(Math.PI);
        redPark2.setZeroPowerAccelerationMultiplier(3.75);
        redPark2.setPathEndTimeoutConstraint(0);
        nextRotate = new Path(new BezierLine(new Point(goPark), new Point(goRotate)));
        nextRotate.setConstantHeadingInterpolation(Math.PI);
        nextRotate.setZeroPowerAccelerationMultiplier(3.75);
        nextRotate.setPathEndTimeoutConstraint(0);
        bitRotate = new Path(new BezierLine(new Point(goRotate), new Point(bitForward)));
        bitRotate.setConstantHeadingInterpolation(Math.PI);
        bitRotate.setZeroPowerAccelerationMultiplier(3.75);
        bitRotate.setPathEndTimeoutConstraint(0);
        toSample2 = new Path(new BezierLine(new Point(bitForward), new Point(bitBack)));
        toSample2.setConstantHeadingInterpolation(Math.PI);
        toSample2.setZeroPowerAccelerationMultiplier(3.75);
        toSample2.setPathEndTimeoutConstraint(0);
        grabSample = new Path(new BezierLine(new Point(bitBack), new Point(toSample)));
        grabSample.setConstantHeadingInterpolation(Math.PI);
        grabSample.setZeroPowerAccelerationMultiplier(3.75);
        grabSample.setPathEndTimeoutConstraint(0);
        nextSample = new Path(new BezierLine(new Point(toSample), new Point(secondScore)));
        nextSample.setConstantHeadingInterpolation(Math.PI);
        nextSample.setZeroPowerAccelerationMultiplier(3.75);
        nextSample.setPathEndTimeoutConstraint(0);
        goingToThirdSample = new Path(new BezierLine(new Point(secondScore), new Point(pushThirdSample)));
        goingToThirdSample.setConstantHeadingInterpolation(Math.PI);
        goingToThirdSample.setZeroPowerAccelerationMultiplier(3.75);
        goingToThirdSample.setPathEndTimeoutConstraint(0);
        sideToThirdSample = new Path(new BezierLine(new Point(pushThirdSample), new Point(pushThirdSampleSide)));
        sideToThirdSample.setConstantHeadingInterpolation(Math.PI);
        sideToThirdSample.setZeroPowerAccelerationMultiplier(3.75);
        sideToThirdSample.setPathEndTimeoutConstraint(0);
        pushingThirdSample = new Path(new BezierLine(new Point(pushThirdSampleSide), new Point(pushThirdSampleBack)));
        pushingThirdSample.setConstantHeadingInterpolation(Math.PI);
        pushingThirdSample.setZeroPowerAccelerationMultiplier(3.75);
        pushingThirdSample.setPathEndTimeoutConstraint(0);
        getCloser = new Path(new BezierLine(new Point(pushThirdSampleBack), new Point(bitCloser)));
        getCloser.setConstantHeadingInterpolation(Math.PI);
        getCloser.setZeroPowerAccelerationMultiplier(3.75);
        getGetBack = new Path(new BezierLine(new Point(bitCloser), new Point(bitBitBack)));
        getGetBack.setConstantHeadingInterpolation(Math.PI);
        getGetBack.setZeroPowerAccelerationMultiplier(3.75);
        toSample3 = new Path(new BezierLine(new Point(bitBitBack), new Point(thirdSample)));
        toSample3.setConstantHeadingInterpolation(Math.PI);
        toSample3.setZeroPowerAccelerationMultiplier(3.75);
        grabSample3 = new Path(new BezierLine(new Point(pushThirdSampleBack), new Point(getThirdSample)));
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
        //scoredSample4.setPathEndTimeoutConstraint(0);
        getFifthSample = new Path(new BezierLine(new Point(fourthScoreCloser), new Point(grabFifthSample)));
        getFifthSample.setConstantHeadingInterpolation(Math.PI);
        scoredFifthSample = new Path(new BezierLine(new Point(grabFifthSample), new Point(scoreFifthSample)));
        scoredFifthSample.setConstantHeadingInterpolation(Math.PI);
        endPark = new Path(new BezierLine(new Point(scoreFifthSample), new Point(finalPark)));
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
                robot.depoHslide.setOut();
                follower.followPath(preload);
                robot.vSlides.moveEncoderTo(robot.vSlides.mid, 1);
                robot.clawBigTilt.setOut();
                robot.depoWrist.setSpecimen();
                robot.clawSmallTilt.setFlat();
                step = 1;
                setPathState(11);
                break;
            case 11: //backs away
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(beforeSpecimen)), Math.toRadians(180));
                    robot.claw.setOpen();
                    // waitFor(100);
                    if(pathTimer.milliseconds() > 300 & step == 1) {
                        robot.depoHslide.setInit();
                        step++;
                    }
                    follower.followPath(redPark);
                    if(pathTimer.milliseconds() > 1000 & step == 2) {
                        vslideGoBottom = true;
                        step++;
                    }
                    follower.followPath(redPark);
                    if(pathTimer.milliseconds() > 1600 & step == 3) {
                        robot.claw.setClose();
                        robot.clawBigTilt.setWall();
                        robot.depoWrist.setWall();
                        robot.clawSmallTilt.setWall();
                        robot.intakeTilt.setHigher();
                        step++;
                    }

                    if(pathTimer.milliseconds() > 1800 & step == 4) {
                        robot.claw.setHalfClose();
                        step = 0;
                        pathTimer.reset();
                        setPathState(12);
                    }
                }
                break;
            case 12: //goes a little in front and to the side of first sample
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(backUp)), Math.toRadians(180));
                    follower.followPath(firstSamplePush);
                    setPathState(13);
                }
                break;
            case 13: //goes right in front of sample
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(pushFirstSample)), Math.toRadians(180));
                    follower.followPath(redPark2);
                    setPathState(14);
                }
                break;
            case 14: //goes right in front of sample
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(goPark)), Math.toRadians(180));
                    follower.followPath(nextRotate);
                    setPathState(15);
                }
                break;
            case 15: //pushes first sample back
                if(!follower.isBusy()) {
                    //follower.holdPoint(new BezierPoint(new Point(goRotate)), Math.toRadians(180));
                    follower.followPath(bitRotate);
                    setPathState(16);
                }
                break;
            case 16: //goes in front and to the side of second sample
                if(!follower.isBusy()) {
                    //follower.holdPoint(new BezierPoint(new Point(goRotate)), Math.toRadians(180));
                    follower.followPath(toSample2);
                    setPathState(17);
                }
                break;
            case 17: //goes in front of second sample
                if(!follower.isBusy()) {
                    //follower.holdPoint(new BezierPoint(new Point(goRotate)), Math.toRadians(180));
                    follower.followPath(grabSample);
                    setPathState(18);
                }
                break;
            case 18: //pushes second sample back
                if(!follower.isBusy()) {
                    //follower.holdPoint(new BezierPoint(new Point(toSample)), Math.toRadians(180));
                    follower.followPath(nextSample);
                    setPathState(19);
                }
                break;
            case 19: //pushes second sample back
                if(!follower.isBusy()) {
                    //follower.holdPoint(new BezierPoint(new Point(toSample)), Math.toRadians(180));
                    follower.followPath(goingToThirdSample);
                    setPathState(20);
                }
                break;
            case 20: //pushes second sample back
                if(!follower.isBusy()) {
                    //follower.holdPoint(new BezierPoint(new Point(toSample)), Math.toRadians(180));
                    follower.followPath(sideToThirdSample);
                    setPathState(21);
                }
                break;
            case 21: //pushes second sample back
                if(!follower.isBusy()) {
                    //follower.holdPoint(new BezierPoint(new Point(toSample)), Math.toRadians(180));
                    follower.followPath(pushingThirdSample);
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
            case 25: //grabs second spec
                if(!follower.isBusy()) {
                    waitFor(400);
                    follower.holdPoint(new BezierPoint(new Point(secondScore)), Math.toRadians(180));
                    follower.followPath(grabSample3);
                    setPathState(26);
                }
                break;
            case 26: //scores second spec
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(getThirdSample)), Math.toRadians(180));
                    robot.claw.setMoreClose();
                    waitFor(100);
                    robot.vSlides.moveEncoderTo(80, 1f);
                    step=1;
                    setPathState(27);
                }
                break;
            case 27: //brings slides up
                if(!follower.isBusy()) {
                    follower.followPath(scoreSample3);
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid, 1f);
                    robot.claw.setClose();
                    robot.clawBigTilt.setOut();
                    robot.depoWrist.setSpecimen();
                    if(pathTimer.milliseconds() > 10 & step == 1) {
                        robot.clawSmallTilt.setFlat();
                        step++;
                    }
                    if(pathTimer.milliseconds() > 50 & step == 2) {
                        robot.clawBigTilt.setOutHigher();
                        robot.depoWrist.setSpecimenHigher();
                        step=0;
                        pathTimer.reset();
                        setPathState(28);
                    }
                }
                break;
            case 28: //
                if(!follower.isBusy()){
                    follower.holdPoint(new BezierPoint(new Point(thirdScore)), Math.toRadians(180));
                    robot.claw.setHalfClose();
                    waitFor(50);
                    follower.followPath(scoredSample3);
                    /*if(pathTimer.milliseconds() > 170 & step == 1) {*/
                    vslideGoBottom = true;
                    robot.clawBigTilt.setTransfer();
                    robot.depoWrist.setTransfer();
                    robot.clawSmallTilt.setTransfer();
                    waitFor(100);
                    robot.clawBigTilt.setWall();
                    robot.depoWrist.setWall();
                    robot.clawSmallTilt.setWall();
                    setPathState(29);
                    /*step = 0;
                    pathTimer.reset();*/
                  /*  }*/
                    /*if(pathTimer.milliseconds() > 200 & step == 3) {
                        robot.claw.setHalfClose();
                        step = 0;
                        pathTimer.reset();
                        setPathState(29);
                    }*/
                }
                break;
            case 29:
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(thirdScoreCloser)), Math.toRadians(180));
                    robot.claw.setMoreClose();
                    waitFor(100);
                    robot.vSlides.moveEncoderTo(80, 1f);
                    step=1;
                   // robot.vSlides.moveEncoderTo(80, 1f);
                    setPathState(30);
                }
                break;
            case 30:
                if(!follower.isBusy()) {
                    follower.followPath(scoreSample4);
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid, 1f);
                    robot.clawBigTilt.setOut();
                    robot.depoWrist.setSpecimen();
                    if(pathTimer.milliseconds() > 10 & step == 1) {
                        robot.clawSmallTilt.setFlat();
                        step++;
                    }
                    if(pathTimer.milliseconds() > 40 & step == 2) {
                        robot.clawBigTilt.setOutHigher();
                        robot.depoWrist.setSpecimenHigher();
                        step=0;
                        pathTimer.reset();
                        setPathState(31);
                    }
                }
                break;
            case 31:
                if(!follower.isBusy()){
                    follower.holdPoint(new BezierPoint(new Point(fourthScore)), Math.toRadians(180));
                    robot.claw.setHalfClose();
                    waitFor(50);
                    follower.followPath(grabSample4);
                    vslideGoBottom = true;
                    robot.clawBigTilt.setTransfer();
                    robot.depoWrist.setTransfer();
                    robot.clawSmallTilt.setTransfer();
                    waitFor(100);
                    robot.clawBigTilt.setWall();
                    robot.depoWrist.setWall();
                    robot.clawSmallTilt.setWall();
                    setPathState(32);
                }
                break;
            case 32:
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(grabFourthSample)), Math.toRadians(180));
                    robot.claw.setMoreClose();
                    waitFor(100);
                    robot.vSlides.moveEncoderTo(80, 1f);
                    step=1;
                    setPathState(33);
                }
                break;
            case 33:
                if(!follower.isBusy()) {
                    follower.followPath(scoredSample4);
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid-20, 1f);
                    robot.claw.setClose();
                    robot.clawBigTilt.setOut();
                    robot.depoWrist.setSpecimen();
                    if(pathTimer.milliseconds() > 20 & step == 1) {
                        robot.clawSmallTilt.setFlat();
                        step++;
                    }
                    if(pathTimer.milliseconds() > 30 & step == 2) {
                        robot.clawBigTilt.setOutHigher();
                        robot.depoWrist.setSpecimenHigher();
                        step=0;
                        pathTimer.reset();
                        setPathState(34);
                    }
                }
                break;
            case 34:
                if(!follower.isBusy()){
                    follower.holdPoint(new BezierPoint(new Point(fourthScoreCloser)), Math.toRadians(180));
                    robot.claw.setHalfClose();
                    waitFor(50);
                    follower.followPath(getFifthSample);
                    vslideGoBottom = true;
                    robot.clawBigTilt.setTransfer();
                    robot.depoWrist.setTransfer();
                    robot.clawSmallTilt.setTransfer();
                    waitFor(100);
                    robot.clawBigTilt.setWall();
                    robot.depoWrist.setWall();
                    robot.clawSmallTilt.setWall();
                    setPathState(35);
                }
                break;
            case 35:
                if(!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(grabFifthSample)), Math.toRadians(180));
                    robot.claw.setMoreClose();
                    waitFor(100);
                    robot.vSlides.moveEncoderTo(80, 1f);
                    step=1;
                    setPathState(36);
                }
                break;
            case 36:
                if(!follower.isBusy()) {
                    follower.followPath(scoredFifthSample);
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid-20, 1f);
                    robot.claw.setClose();
                    robot.clawBigTilt.setOut();
                    robot.depoWrist.setSpecimen();
                    if(pathTimer.milliseconds() > 20 & step == 1) {
                        robot.clawSmallTilt.setFlat();
                        step++;
                    }
                    if(pathTimer.milliseconds() > 30 & step == 2) {
                        robot.clawBigTilt.setOutHigher();
                        robot.depoWrist.setSpecimenHigher();
                        step=0;
                        pathTimer.reset();
                        setPathState(37);
                    }
                }
                break;
            case 37:
                if(!follower.isBusy()){
                    follower.holdPoint(new BezierPoint(new Point(scoreFifthSample)), Math.toRadians(180));
                    step=1;
                    robot.claw.setHalfClose();
                    waitFor(50);
                    follower.followPath(endPark);
                    if(pathTimer.milliseconds() > 170 & step == 1) {
                        vslideGoBottom = true;
                        robot.claw.setOpen();
                        robot.clawBigTilt.setTransfer();
                        robot.depoWrist.setTransfer();
                        robot.clawSmallTilt.setTransfer();
                        step = 0;
                        pathTimer.reset();
                        setPathState(100);
                    }
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

        //functions
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
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);


        //Pose init
        firstSpecimen();
        buildPaths();

        //robot init
        robot.intakeTilt.setInOut();
        robot.clawBigTilt.setOut();
        robot.depoWrist.setSpecimen();
        robot.clawSmallTilt.setFlat();
        robot.claw.setClose();

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

