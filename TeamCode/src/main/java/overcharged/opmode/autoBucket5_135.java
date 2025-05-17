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

// Main Class
@Autonomous(name = "0+5 red bucket test", group = "0Autonomous")
public class autoBucket5_135 extends OpMode{

    //stuff
    boolean vslideGoBottom = false;
    boolean hSlideGoBottom = false;
    boolean scored = false;
    boolean in = true;
    boolean broken = false;
    boolean nowDelay = false;
    boolean backDelay = false;
    boolean s2Delay = false;
    boolean runOnce = false;
    boolean doOnce = false;
    boolean canScore = false;
    boolean trappy = false;
    boolean runningOutOfNames = false;
    boolean canSub = false;

    ElapsedTime totalTime;
    long tempTime;


    int floorRep = 3;
    int tempWait = 0;

    // Init
    private RobotMecanum robot;
    private DigitalChannel hlimitswitch;
    private DigitalChannel vlimitswitch;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SampleMecanumDrive drive;
    MultipleTelemetry telems;
    private ElapsedTime pathTimer, delayTimer, secTimer, thiTimer;

    // Other init
    private int pathState;
    int moveCoEff = 0;

    // LOCATIONS
    // GUIDE:
    // (0,0) is the corner. (144, 144) is the opposite.

    // OTHER POSES
    private Pose initBucket, beforeBucket, ready2Score, wallScore, iShouldGetCloserToWallBecauseMyRobotKeepsOnDying, beforeBucket2, subFront, beforeBucket3;
    private Pose startPose = new Pose(137, 31, Math.toRadians(90));

    private Path firstScore, inchBucket, goSafe, goBack, secondBack, floor2, slightMove, tinyFloorMove, thirdBack, floor3, breakControl1, breakControl2;

    private PathChain preload, floorCycle, toSub, backSub;

    private Follower follower;

    //TODO: Starting from here are the poses for the paths
    public void firstBucket(){
        beforeBucket = new Pose(121,20.5, Math.PI);
        beforeBucket2 = new Pose(122.5,12.5, Math.PI);
        beforeBucket3 = new Pose(124,12.5, Math.PI);
        ready2Score = new Pose(133.3,14.5,Math.toRadians(135));
        wallScore = new Pose(128.8,8.0, Math.PI);
        iShouldGetCloserToWallBecauseMyRobotKeepsOnDying = new Pose(125.40,13.55, Math.PI);
        subFront = new Pose(82.000, 49.500, Math.PI/2);
    }

    //TODO: here are where the paths are defined
    public void buildPaths() {

        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose),new Point(ready2Score)))
                .setLinearHeadingInterpolation(startPose.getHeading(), ready2Score.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        goSafe = new Path(new BezierLine(new Point(ready2Score), new Point(beforeBucket)));

        secondBack = new Path(new BezierLine(new Point(wallScore), new Point(beforeBucket2)));
        secondBack.setConstantHeadingInterpolation(Math.PI);

        thirdBack = new Path(new BezierLine(new Point(wallScore), new Point(beforeBucket3)));
        thirdBack.setConstantHeadingInterpolation(Math.PI);

        floor2 = new Path(new BezierLine(new Point(beforeBucket2), new Point(wallScore)));
        floor2.setConstantHeadingInterpolation(Math.PI);
        floor2.setZeroPowerAccelerationMultiplier(3);

        floor3 = new Path(new BezierLine(new Point(beforeBucket3), new Point(wallScore)));
        floor3.setConstantHeadingInterpolation(Math.PI);
        floor3.setZeroPowerAccelerationMultiplier(3);


        floorCycle = follower.pathBuilder()
                .addPath(new BezierLine(new Point(beforeBucket), new Point(wallScore)))
                .setConstantHeadingInterpolation(Math.PI)
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        tinyFloorMove = new Path(new BezierLine(new Point(wallScore), new Point(iShouldGetCloserToWallBecauseMyRobotKeepsOnDying)));
        tinyFloorMove.setConstantHeadingInterpolation(Math.PI);

        toSub = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(wallScore),
                        new Point(100.000, 22.000, Point.CARTESIAN),
                        new Point(80.000, 25.000, Point.CARTESIAN),
                        new Point(subFront)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .setZeroPowerAccelerationMultiplier(3.25)
                .build();

        backSub = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(subFront.getX()+(moveCoEff)*2, subFront.getY()),
                        new Point(80.000, 25.000, Point.CARTESIAN),
                        new Point(100.000, 22.000, Point.CARTESIAN),
                        new Point(wallScore)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3.25)
                .build();

        slightMove = new Path(new BezierLine(new Point(subFront.getX()+(moveCoEff - 1)*2, subFront.getY()), new Point(subFront.getX()+(moveCoEff)*2, subFront.getY())));

        breakControl1 = new Path(new BezierLine(new Point(beforeBucket), new Point(beforeBucket2)));
        breakControl1.setConstantHeadingInterpolation(Math.toRadians(180));

        breakControl2 = new Path(new BezierLine(new Point(beforeBucket2), new Point(beforeBucket3)));
        breakControl2.setConstantHeadingInterpolation(Math.toRadians(180));
    }


    // TODO: HERE IS WHERE THE MAIN PATH IS
    // Main pathing
    public void autoPath() {
        switch (pathState) {
            // Auto Body
            //
            case 10: // scores initial specimen
                robot.hslides.hslides.resetPosition();
                robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.intakeTilt.setFlat();
                robot.vSlides.moveEncoderTo(robot.vSlides.autohigh1+20, 1f);
                follower.followPath(preload, true);
                setPathState(12);
                break;
            case 12:
                if(pathTimer.milliseconds()>400){
                    robot.latch.setOut();
                    robot.hslides.moveEncoderTo(robot.hslides.SMALL, 0.7f);
                    setPathState(13);
                    runOnce = true;
                }
                break;
            case 13:
                if(pathTimer.milliseconds()>200 && (robot.vSlides.vSlidesL.getCurrentPosition()>470 || pathTimer.milliseconds()>1400)) {
                    if (runOnce) {
                        runOnce = false;
                        robot.clawSmallTilt.setOut();
                        robot.clawBigTilt.setBucket();
                        robot.depoWrist.setBucket();
                        robot.depoHslide.setInit();
                        delayTimer.reset();
                        nowDelay = true;
                    }
                    if (delayTimer.milliseconds() > 200 && nowDelay){
                        canScore = true;
                        nowDelay = false;
                        doOnce = true;
                    }
                    if ((follower.getPose().getX() > (ready2Score.getX() - 1.4) && follower.getPose().getY() > (ready2Score.getY() - 2.15) && Math.abs(robot.vSlides.vSlidesL.getCurrentPosition()-825) < 16 && canScore)|| pathTimer.milliseconds()>2500) {
                        if (doOnce) {
                            doOnce = false;
                            canScore = false;
                            backDelay = true;
                            setPathState(130);
                        }
                    }
                }
                break;
            case 130:
                if (pathTimer.milliseconds() > 380 && backDelay){
                    robot.claw.setBig();
                }
                if (pathTimer.milliseconds() > 780 && backDelay) {
                    backDelay = false;
                    robot.clawBigTilt.setTransfer();
                    robot.depoWrist.setTransfer();
                    robot.clawSmallTilt.setTransfer();
                    runOnce = true;
                    canScore = false;
                    setPathState(14);
                }
                break;
            case 14:
                robot.claw.setOpen();
                if(floorRep < 1){
                    setPathState(18);
                    runOnce = true;
                }
                else if (broken){
                    broken = false;
                    setPathState(141);
                }
                else if (!broken) {
                    setPathState(142);
                }
                break;
            case 141:
                robot.latch.setOut();
                robot.intakeTilt.setOut();
                robot.intake.in();
                if(pathTimer.milliseconds() > 3000 && hSlideGoBottom && !hlimitswitch.getState()) {
                    hSlideGoBottom = false;
                    robot.hslides.hslides.setPower(0);
                    robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                    floorRep -= 1;
                    setPathState(1410);

                break;
            case 1410:
                scored = false;
                runOnce = true;
                if (floorRep == 2) {
                    follower.followPath(breakControl1);
                    follower.setMaxPower(0.7f);
                } else if (floorRep == 1) {
                    follower.followPath(breakControl2);
                    follower.setMaxPower(0.7f);
                } else {
                    telemetry.addLine("aw2 man2");
                }
                setPathState(16);
                break;
            case 142:
                scored = false;
                runOnce = true;
                robot.intakeTilt.setOut();
                robot.intake.in();
                if (floorRep == 3) {
                    follower.followPath(goSafe);
                    goSafe.setLinearHeadingInterpolation(ready2Score.getHeading(), Math.toRadians(180));
                } else if (floorRep == 2) {
                    follower.followPath(secondBack);
                } else if (floorRep == 1) {
                    follower.followPath(thirdBack);
                    thirdBack.setLinearHeadingInterpolation(wallScore.getHeading(), Math.toRadians(206));
                } else {
                    telemetry.addLine("aw man");
                }
                setPathState(16);
                break;
            case 16:
                if(pathTimer.milliseconds()>300){
                    if (runOnce) {
                        runOnce = false;
                        robot.depoWrist.setTransfer();
                        robot.clawBigTilt.setTransfer();
                        robot.clawSmallTilt.setTransfer();
                        vslideGoBottom = true;
                        setPathState(160);
                    }
                }
                break;
            case 160:
                if(!follower.isBusy()) {
                    in = false;
                    runOnce = true;
                    robot.intake.in();
                    if (floorRep == 3){
                        robot.hslides.moveEncoderTo(robot.hslides.PRESET1, 1f);
                    } else if (floorRep == 2){
                        robot.hslides.moveEncoderTo(robot.hslides.PRESET1, 1f);
                    } else if (floorRep == 1){
                        robot.hslides.moveEncoderTo(robot.hslides.PRESET2, 1f);
                    }
                    setPathState(161);
                }
                break;
            case 161:
                if(robot.sensorF.getColor() == colorSensor.Color.YELLOW){
                    tempTime = System.currentTimeMillis();
                    if (runOnce){
                        runOnce = false;
                        robot.intakeTilt.setTransfer();
                        robot.intake.in();
                    }
                    if(tempTime > 300) {
                        hSlideGoBottom = true;
                        setPathState(17);
                        tempTime = 0;
                        runOnce = true;

                    }
                }
                else if(robot.sensorF.getColor() == colorSensor.Color.NONE && pathTimer.milliseconds()<1900) {
                    robot.latch.setOut();
                    if (broken){
                        broken = false;
                        if (floorRep >1){
                            robot.hslides.moveEncoderTo(robot.hslides.PRESET1, 1f);
                        } else if (floorRep == 1){
                            robot.hslides.moveEncoderTo(robot.hslides.PRESET1, 1f);
                        }
                    } else{
                        robot.hslides.moveEncoderTo(robot.hslides.hslides.getCurrentPosition() + 90, 1f);
                    }
                }
                else if (robot.sensorF.getColor() == colorSensor.Color.NONE && pathTimer.milliseconds()>1910) {
                    robot.intake.out();
                    robot.latch.setOut();
                    robot.intakeTilt.setTransfer();
                    hSlideGoBottom = true;
                    scored = true;
                    broken = true;
                    setPathState(14);
                }

                break;
            case 17:
                if(pathTimer.milliseconds()>700) {
                    robot.intake.out();
                }
                if (in) {
                    follower.setMaxPower(1f);
                    if (runOnce) {
                        runOnce = false;
                        if (floorRep == 3) {
                            follower.followPath(floorCycle);
                        } else if (floorRep == 2) {
                            follower.followPath(floor2);
                        }
                        else if (floorRep == 1) {
                            follower.followPath(floor3);
                            floor3.setLinearHeadingInterpolation(Math.toRadians(206), Math.PI);
                        }
                        backDelay = true;
                    }
                    //delay claw close once hslides are back
                    if (!hSlideGoBottom && backDelay) {
                        setPathState(1700);
                        backDelay = false;
                    }
                }
                break;
            case 1700:
                if(pathTimer.milliseconds()>350) {
                    robot.claw.setClose();
                    robot.intake.off();
                    nowDelay = true;
                    runOnce = true;
                    setPathState(170);
                }
                break;
            case 170:
                //vslide up delay
                if (pathTimer.milliseconds()>200 && nowDelay) {
                    if (runOnce) {
                        runOnce = false;
                        nowDelay = false;
                        robot.intakeTilt.setFlat();
                        robot.vSlides.moveEncoderTo(robot.vSlides.autohigh1, 1f);
                        s2Delay = true;
                    }
                }
                if (s2Delay && robot.vSlides.vSlidesL.getCurrentPosition()>470) {
                    s2Delay = false;
                    robot.clawBigTilt.setBucket();
                    robot.depoWrist.setBucket();
                    setPathState(171);
                }


                break;
            case 171:
                if(pathTimer.milliseconds()>200){
                    robot.depoHslide.setInit();
                    robot.clawSmallTilt.setRight();
                    robot.intakeTilt.setTransfer();
                    runOnce = true;
                    setPathState(1710);
                }
                break;
            case 1710:
                if((pathTimer.milliseconds() > 200 && follower.getPose().getX() > (wallScore.getX() - 1.2) && follower.getPose().getY() > (wallScore.getY() - 1.8) && Math.abs(robot.vSlides.vSlidesL.getCurrentPosition()-805) < 18)|| pathTimer.milliseconds()>1500){
                    if (runOnce) {
                        runOnce = false;
                        robot.latch.setOut();
                        robot.hslides.moveEncoderTo(robot.hslides.SMALL+10, 1f);
                        setPathState(1711);
                    }
                }
                break;
            case 1711:
                if(pathTimer.milliseconds()>300) {
                    robot.claw.setBig();
                    scored = true;
                    setPathState(172);
                }
                break;
            case 172:
                if(pathTimer.milliseconds() > 50) {
                    if (floorRep > 1) {
                        floorRep -= 1;
                        setPathState(14);
                    } else {
                        floorRep -= 1;
                        setPathState(18);
                        runOnce = true;
                    }
                }
                break;

            case 18:
                if(delayTimer.milliseconds()>50) {
                    if (runOnce) {
                        runOnce = false;
                        delayTimer.reset();
                        nowDelay = true;
                        secTimer.reset();
                    }
                }
                if (secTimer.milliseconds()>420 && nowDelay) {
                    secTimer.reset();
                    nowDelay = false;
                    robot.claw.setOpen();
                    robot.clawBigTilt.setTransfer();
                    robot.depoWrist.setTransfer();
                    robot.clawSmallTilt.setTransfer();
                    setPathState(19);
                }
                break;
            case 19:
                if(follower.getCurrentTValue()>0.85){
                    follower.followPath(toSub, true);
                    robot.intakeTilt.setTransfer();
                    runOnce = true;
                    setPathState(191);
                }
                break;
            case 191:
                if (follower.getCurrentTValue()>0.3) {
                    if (runOnce) {
                        runOnce = false;
                        vslideGoBottom = true;
                    }
                }
                if (follower.getCurrentTValue()>0.8) {
                    robot.latch.setOut();
                    robot.hslides.moveEncoderTo(hslides.SMALL+30, 1f);
                    setPathState(20);
                    runOnce = true;
                    canScore = true;
                }
                break;
            case 20: // SUB CODE
                //TODO: COMMENT NEXT LINES OUT
                //robot.latch.setOut();
                //robot.hslides.moveEncoderTo(hslides.SMALL, 1f);
                if (runOnce) {
                    runOnce = false;
                    if (!canScore) {
                        follower.followPath(slightMove, true);
                        slightMove.setConstantHeadingInterpolation(Math.toRadians(90));
                    } else {
                        canScore = false;
                    }
                    setPathState(201);
                }
                break;
            case 201:
                if(robot.hslides.hslides.getCurrentPosition()<50) {
                    hSlideGoBottom = false;
                    robot.latch.setOut();
                    robot.hslides.moveEncoderTo(robot.hslides.SMALL+35, 0.8f);
                    setPathState(202);
                }
            case 202:
                if(pathTimer.milliseconds()>600) {
                    robot.trapdoor.setInit();
                    robot.intake.in();
                    robot.intakeTilt.setOut();
                    canSub = true;
                    setPathState(21);
                }
                break;
            case 21:
                if(robot.hslides.hslides.getCurrentPosition()<600 && canSub) {
                    robot.hslides.moveEncoderTo( robot.hslides.hslides.getCurrentPosition() + 60, 1f);
                    if (trappy){
                        robot.intake.in();
                        robot.trapdoor.setInit();
                        trappy = false;
                        runningOutOfNames = true;
                    }
                }else if (robot.hslides.hslides.getCurrentPosition()>605 && canSub){
                    moveCoEff += 1;
                    robot.trapdoor.setInit();
                    setPathState(211);
                }
                if(robot.sensorF.getColor() == colorSensor.Color.RED || robot.sensorF.getColor() == colorSensor.Color.YELLOW){
                    canSub = false;
                    robot.intakeTilt.setTransfer();
                    setPathState(22);
                }
                else if (robot.sensorF.getColor() == colorSensor.Color.BLUE && runningOutOfNames){
                    canSub = false;
                    robot.trapdoor.setOut();
                    robot.hslides.moveEncoderTo(robot.hslides.hslides.getCurrentPosition()+50,1f);
                    robot.intake.off();
                    runningOutOfNames = false;
                    trappy = true;
                }
                else if (pathTimer.milliseconds()>1400){
                    canSub = false;
                    moveCoEff += 1;
                    setPathState(211);
                }
                break;
            case 211:
                if(pathTimer.milliseconds() > 400) {
                    robot.intake.out();
                    setPathState(220);
                }
                break;
            case 220:
                if(pathTimer.milliseconds() > 160) {
                    robot.intake.in();
                    runOnce = true;
                    hSlideGoBottom = true;
                    setPathState(20);
                }
                break;
            case 22:
                if(pathTimer.milliseconds()>250) {
                    robot.intake.out();
                    hSlideGoBottom = true;
                    runningOutOfNames = false;
                    setPathState(23);
                    //setPathState(100);
                }
            case 23: //SUB FINISH
                if(pathTimer.milliseconds()>200 && follower.getCurrentTValue() > 0.9) {
                    hSlideGoBottom = true;
                    follower.followPath(backSub, true);
                    robot.intake.out();
                    s2Delay = false;
                    nowDelay = true;
                    runOnce = true;
                    setPathState(24);
                }
                break;
            case 24:
                if(pathTimer.milliseconds()>400 && nowDelay && robot.hslides.hslides.getCurrentPosition() < 50){
                    if(runOnce) {
                        robot.intake.off();
                        runOnce = false;
                        robot.claw.setClose();
                        nowDelay = false;
                        canScore = true;
                        doOnce = true;
                    }
                }
                if(pathTimer.milliseconds()>400) {
                    hSlideGoBottom = true;
                }
                if(pathTimer.milliseconds()>580 && canScore){
                    if(doOnce) {
                        doOnce = false;
                        s2Delay = true;
                        robot.vSlides.moveEncoderTo(robot.vSlides.autohigh1+30, 1f);
                        setPathState(2401);
                    }
                }
                break;
            case 2401:
                if(pathTimer.milliseconds()>100) {
                    robot.intakeTilt.setFlat();
                }
                if (pathTimer.milliseconds()>200) {
                    setPathState(241);
                }
                break;
            case 241:
                if(follower.getPose().getX() > (wallScore.getX() - 52) && follower.getPose().getY() < (wallScore.getY() + 32)){
                    robot.clawBigTilt.setBucket();
                    robot.depoWrist.setBucket();
                    robot.depoHslide.setInit();
                    runningOutOfNames = true;
                }
                if(pathTimer.milliseconds()>200 && robot.vSlides.vSlidesL.getCurrentPosition() > 480 && runningOutOfNames){
                    robot.clawSmallTilt.setRight();
                    robot.intakeTilt.setTransfer();
                    canScore = true;
                    runOnce = true;
                }
                if((pathTimer.milliseconds()>200 && canScore && follower.getPose().getX() > (wallScore.getX() - 1.0) && follower.getPose().getY() > (wallScore.getY() - 1.8) && Math.abs(robot.vSlides.vSlidesL.getCurrentPosition()-835) < 15)|| pathTimer.milliseconds()>8000){
                    if (runOnce) {
                        runOnce = false;
                        canScore = false;
                        setPathState(2410);
                    }
                }
                break;
            case 2410:
                if(pathTimer.milliseconds() > 400) {
                    robot.claw.setBig();
                    setPathState(242);
                }
                break;
            case 242:
                if(totalTime.milliseconds()<24000){
                    setPathState(18);
                } else{
                    setPathState(25);
                }
                break;
            case 25:
                if(pathTimer.milliseconds()>400){
                    robot.clawBigTilt.setTransfer();
                    robot.depoWrist.setTransfer();
                    robot.claw.setOpen();
                }
                if(pathTimer.milliseconds()>750){
                    vslideGoBottom = true;
                    setPathState(100);
                }
                break;

            case 100: // EMPTY TEST CASE
                telems.addLine("CASE 100 - IN TEST CASE!!");
                break;

        }
    }


    // path setter
    public void setPathState(int state){
        pathState = state;
        pathTimer.reset();
        delayTimer.reset();
        secTimer.reset();
        thiTimer.reset();
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
        telemetry.addLine("TValue: " + follower.getCurrentTValue());
        telemetry.addLine("Case: " + pathState);
        telemetry.addLine("Position: " + follower.getPose());
        telemetry.addLine("heading: " + follower.getTotalHeading());

        telemetry.addLine("total time: " + totalTime);

        //telemetry.addLine("Vslide pos: " + robot.vSlides.vSlidesL.getCurrentPosition());
        //telemetry.addLine("Straight encoder: ");
        //telemetry.addLine("color: "+robot.sensorF.getColor());
        //telemetry.addLine("vLimit" + vlimitswitch.getState());
        //telemetry.addLine("hLimit" + hlimitswitch.getState());
        telemetry.addLine("Rep Count: " + floorRep);
        telemetry.addLine("pathTimer: " + pathTimer.milliseconds());
        //telemetry.addLine("delayTimer: " + delayTimer);
        //telemetry.addLine("secTimer: " + secTimer);
        //telemetry.addLine("thiTimer: " + thiTimer);
        //telemetry.addLine("delayTimer: " + pathTimer.milliseconds());
        /*telemetry.addLine("doOnce: " + doOnce);
        telemetry.addLine("backDelay: " + backDelay);
        telemetry.addLine("s2Delay: " + s2Delay);
        telemetry.addLine("nowDelay: " + nowDelay);

         */
        telemetry.addLine("hslideGoBottom: " + hSlideGoBottom);
        //telemetry.addLine("vslides: " + robot.vSlides.vSlidesR.getCurrentPosition());
        telemetry.addLine("hslides: " + robot.hslides.hslides.getCurrentPosition());
        //telemetry.addLine("color: " + robot.sensorF.getColor());
        //functions
        if (hSlideGoBottom) {
            if (!hlimitswitch.getState()) {
                in = true;
                robot.latch.setInit();
                robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.hslides.hslides.setPower(-1f);
                RobotLog.ii(TAG_SL, "Going down");
            } else if (hlimitswitch.getState()) {
                robot.latch.setInit();
                robot.intakeTilt.setTransfer();
                robot.hslides.hslides.setPower(0);
                robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hSlideGoBottom = false;
                RobotLog.ii(TAG_SL, "Force stopped");
            }
        }

        if (vslideGoBottom) {
            robot.claw.setClose();
            if (!vlimitswitch.getState()) {
                robot.vSlides.vSlidesL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.vSlides.vSlidesR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.vSlides.vSlidesR.setPower(-0.8f);
                robot.vSlides.vSlidesL.setPower(-0.8f);
            } else if (vlimitswitch.getState()) {
                robot.claw.setOpen();
                robot.vSlides.vSlidesR.setPower(0);
                robot.vSlides.vSlidesL.setPower(0);
                robot.vSlides.vSlidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.vSlides.vSlidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                vslideGoBottom = false;
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
        delayTimer = new ElapsedTime();
        pathTimer = new ElapsedTime();
        secTimer = new ElapsedTime();
        thiTimer = new ElapsedTime();


        //follower init
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        //Pose init
        firstBucket();
        buildPaths();

        //robot init
        hlimitswitch = hardwareMap.get(DigitalChannel.class, "hlimitswitch");
        vlimitswitch = hardwareMap.get(DigitalChannel.class, "vlimitswitch");
        robot.intakeTilt.setTransfer();
        robot.clawBigTilt.setTransfer();
        robot.clawSmallTilt.setTransfer();
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

        //timer
        totalTime = new ElapsedTime();
    }

    public static void waitFor(int milliseconds) { //Waitor Function
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < milliseconds) {
            // loop
        }
    }
}