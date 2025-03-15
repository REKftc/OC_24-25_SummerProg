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

    long totalTime;
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
    private Pose initBucket, beforeBucket, ready2Score, wallScore, iShouldGetCloserToWallBecauseMyRobotKeepsOnDying, beforeBucket2, subFront;
    private Pose startPose = new Pose(132, 36, Math.toRadians(90));

    private Path firstScore, inchBucket, goSafe, goBack, secondBack, floor2, slightMove, tinyFloorMove;

    private PathChain preload, floorCycle, toSub, backSub;

    private Follower follower;

    //TODO: Starting from here are the poses for the paths
    public void firstBucket(){
        beforeBucket = new Pose(120,24.85, Math.PI);
        beforeBucket2 = new Pose(120,16.8, Math.PI);
        ready2Score = new Pose(127.75,19.50,Math.toRadians(135));
        wallScore = new Pose(124.15,14.45, Math.PI);
        iShouldGetCloserToWallBecauseMyRobotKeepsOnDying = new Pose(125.40,13.55, Math.PI);
        subFront = new Pose(82.000, 51.500, Math.PI/2);
    }

    //TODO: here are where the paths are defined
    public void buildPaths() {

        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose),new Point(ready2Score)))
                .setLinearHeadingInterpolation(startPose.getHeading(), ready2Score.getHeading())
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();

        goSafe = new Path(new BezierLine(new Point(ready2Score), new Point(beforeBucket)));

        secondBack = new Path(new BezierLine(new Point(wallScore), new Point(beforeBucket2)));
        secondBack.setConstantHeadingInterpolation(Math.PI);

        floor2 = new Path(new BezierLine(new Point(beforeBucket2), new Point(wallScore)));
        floor2.setConstantHeadingInterpolation(Math.PI);
        floor2.setZeroPowerAccelerationMultiplier(3);


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
                        new Point(125.000, 30.000, Point.CARTESIAN),
                        new Point(75.000, 10.000, Point.CARTESIAN),
                        new Point(subFront)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .setZeroPowerAccelerationMultiplier(3.25)
                .build();

        backSub = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(subFront.getX()+(moveCoEff)*2, subFront.getY()),
                        new Point(75.000, 10.000, Point.CARTESIAN),
                        new Point(125.000, 30.000, Point.CARTESIAN),
                        new Point(wallScore)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3.25)
                .build();

        slightMove = new Path(new BezierLine(new Point(subFront.getX()+(moveCoEff - 1)*1.35, subFront.getY()), new Point(subFront.getX()+(moveCoEff)*1.35, subFront.getY())));
    }


    // TODO: HERE IS WHERE THE MAIN PATH IS
    // Main pathing
    public void autoPath() {
        switch (pathState) {
            // Auto Body
            //
            case 10: // scores initial specimen
                robot.intakeTilt.setFlat();
                robot.vSlides.moveEncoderTo(robot.vSlides.autohigh1, 1f);
                follower.followPath(preload, true);
                setPathState(12);
                nowDelay = false;
                break;
            case 12:
                if(pathTimer.milliseconds()>50){
                    nowDelay = true;
                    if(delayTimer.milliseconds()>190 && nowDelay){
                        nowDelay = false;
                        robot.latch.setOut();
                        robot.hslides.moveEncoderTo(robot.hslides.SMALL, 1f);
                        setPathState(13);
                        runOnce = true;
                    }
                }
                break;
            case 13:
                if(pathTimer.milliseconds()>200 && robot.vSlides.vSlidesL.getCurrentPosition()>470) {
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
                    if (follower.getPose().getX() > (ready2Score.getX() - 1) && follower.getPose().getY() > (ready2Score.getY() - 2.45) && Math.abs(robot.vSlides.vSlidesL.getCurrentPosition()-845) < 30 && canScore) {
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
                if (pathTimer.milliseconds() > 400 && backDelay){
                    robot.claw.setBig();
                }
                if (pathTimer.milliseconds() > 520 && backDelay) {
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
                if(floorRep == 0){
                    setPathState(18);
                    runOnce = true;
                }
                if (broken){
                    broken = false;
                    robot.latch.setOut();
                    robot.hslides.moveEncoderTo(robot.hslides.SMALL, 1f);
                    robot.intakeTilt.setOut();
                    if (doOnce){
                        doOnce = false;
                        floorRep -= 1;
                    }
                }
                robot.claw.setOpen();
                nowDelay = false;
                scored=true;
                if (scored) {
                    scored = false;
                    runOnce = true;
                    robot.intakeTilt.setOut();
                    robot.intake.in();
                    if (floorRep == 3) {
                        follower.followPath(goSafe, true);
                        goSafe.setLinearHeadingInterpolation(ready2Score.getHeading(), Math.toRadians(180));
                    } else if (floorRep == 2) {
                        follower.followPath(secondBack);
                    } else if (floorRep == 1) {
                        follower.followPath(secondBack);
                        secondBack.setLinearHeadingInterpolation(wallScore.getHeading(), Math.toRadians(203));
                    } else {
                        telemetry.addLine("aw man");
                    }
                    setPathState(16);
                }

                break;
            case 16:
                if(pathTimer.milliseconds()>350){
                    if (runOnce) {
                        runOnce = false;
                        vslideGoBottom = true;
                    }
                }
                if(!follower.isBusy()) {
                    in = false;
                    runOnce = true;
                    if (floorRep >1){
                        robot.hslides.moveEncoderTo(robot.hslides.PRESET1, 1f);
                    } else if (floorRep == 1){
                        robot.hslides.moveEncoderTo(robot.hslides.PRESET2, 1f);
                    }
                    if (floorRep == 3){
                        follower.holdPoint(beforeBucket);
                    } else if (floorRep == 2){
                        follower.holdPoint(beforeBucket2);
                    } else if (floorRep == 1){
                        follower.holdPoint(new Pose(beforeBucket2.getX(), beforeBucket2.getY(), Math.toRadians(203)));
                    }
                    robot.depoWrist.setTransfer();
                    robot.clawBigTilt.setTransfer();
                    robot.clawSmallTilt.setTransfer();
                    setPathState(161);
                }
                break;
            case 161:
                if(robot.sensorF.getColor() == colorSensor.Color.YELLOW){
                    tempTime = System.currentTimeMillis();
                    if (runOnce){
                        secTimer.reset();
                        runOnce = false;
                        robot.intakeTilt.setTransfer();
                        robot.intake.in();
                    }
                    if(tempTime > 500) {
                        hSlideGoBottom = true;
                        setPathState(17);
                        tempTime =0;
                        nowDelay = false;
                        runOnce = true;

                    }
                }
                else if(robot.sensorF.getColor() == colorSensor.Color.NONE && pathTimer.milliseconds()<1800) {
                    robot.latch.setOut();
                    if (broken){
                        broken = false;
                        if (floorRep >1){
                            robot.hslides.moveEncoderTo(robot.hslides.PRESET1+80, 0.9f);
                        } else if (floorRep == 1){
                            robot.hslides.moveEncoderTo(robot.hslides.PRESET2, 1f);
                        }
                    } else{
                        robot.hslides.moveEncoderTo(robot.hslides.hslides.getCurrentPosition() + 90, 1f);
                    }
                }
                if (robot.sensorF.getColor() == colorSensor.Color.NONE && pathTimer.milliseconds()>1850) {
                    robot.intake.out();
                    robot.latch.setOut();
                    robot.intakeTilt.setTransfer();
                    hSlideGoBottom = true;
                    scored = true;
                    broken = true;
                    runOnce = true;
                    doOnce = true;
                    setPathState(14);
                }

                break;
            case 17:
                if(pathTimer.milliseconds()>700) {
                    robot.intake.out();
                }
                if (in) {
                    if (runOnce) {
                        runOnce = false;
                        if (floorRep == 3) {
                            follower.followPath(floorCycle);
                        } else if (floorRep > 0) {
                            follower.followPath(floor2);
                        }
                        if (floorRep == 1) {
                            floor2.setLinearHeadingInterpolation(Math.toRadians(203), Math.PI);
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
                if(pathTimer.milliseconds()>280) {
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
                if(pathTimer.milliseconds() > 200 && follower.getPose().getX() > (wallScore.getX() - 1.2) && follower.getPose().getY() > (wallScore.getY() - 1.8) && Math.abs(robot.vSlides.vSlidesL.getCurrentPosition()-845) < 24){
                    if (runOnce) {
                        runOnce = false;
                        robot.latch.setOut();
                        robot.hslides.moveEncoderTo(robot.hslides.SMALL, 1f);
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
                if (secTimer.milliseconds()>120 && nowDelay) {
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
                    vslideGoBottom = true;
                    setPathState(191);
                }
                break;
            case 191:
                if (follower.getCurrentTValue()>0.8) {
                    robot.latch.setOut();
                    robot.hslides.moveEncoderTo(hslides.SMALL, 1f);
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
                if(pathTimer.milliseconds()>200) {
                    robot.trapdoor.setInit();
                    robot.intake.in();
                    robot.intakeTilt.setOut();
                    setPathState(21);
                }
            case 21:
                if(robot.hslides.hslides.getCurrentPosition()<600) {
                    robot.hslides.moveEncoderTo((int) robot.hslides.hslides.getCurrentPosition() + 80, 1f);
                    if (trappy){
                        robot.intake.in();
                        robot.trapdoor.setInit();
                        trappy = false;
                        runningOutOfNames = true;
                    }
                }
                else{
                    moveCoEff += 1;
                    robot.trapdoor.setInit();
                    setPathState(211);
                }
                if(robot.sensorF.getColor() == colorSensor.Color.RED || robot.sensorF.getColor() == colorSensor.Color.YELLOW){
                    robot.intakeTilt.setTransfer();
                    setPathState(22);
                }
                else if (robot.sensorF.getColor() == colorSensor.Color.BLUE && runningOutOfNames){
                    robot.trapdoor.setOut();
                    robot.intake.off();
                    runningOutOfNames = false;
                    trappy = true;
                }
                else if (pathTimer.milliseconds()>1400){
                    moveCoEff += 1;
                    setPathState(211);
                }
                break;
            case 211:
                hSlideGoBottom = true;
                if(pathTimer.milliseconds() > 200) {
                    robot.intake.out();
                }
                if(pathTimer.milliseconds() > 360) {
                    robot.intake.in();
                    runOnce = true;
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
                if(follower.getCurrentTValue() > 0.9) {
                    hSlideGoBottom = true;
                    follower.followPath(backSub, true);
                    robot.intake.off();
                    s2Delay = false;
                    nowDelay = true;
                    runOnce = true;
                    setPathState(24);
                }
                break;
            case 24:
                if(pathTimer.milliseconds()>400 && nowDelay && robot.hslides.hslides.getCurrentPosition() < 50){
                    if(runOnce) {
                        runOnce = false;
                        robot.claw.setClose();
                        nowDelay = false;
                        doOnce = true;
                    }
                }
                if(pathTimer.milliseconds()>580){
                    if(doOnce) {
                        doOnce = false;
                        s2Delay = true;
                        robot.vSlides.moveEncoderTo(robot.vSlides.autohigh1, 1f);
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
                if(follower.getPose().getX() > (wallScore.getX() - 40) && follower.getPose().getY() < (wallScore.getY() + 24)){
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
                if(pathTimer.milliseconds()>200 && canScore && follower.getPose().getX() > (wallScore.getX() - 1.2) && follower.getPose().getY() > (wallScore.getY() - 2.0) && Math.abs(robot.vSlides.vSlidesL.getCurrentPosition()-845) < 25){
                    if (runOnce) {
                        runOnce = false;
                        backDelay = true;
                        canScore = false;
                        thiTimer.reset();
                        setPathState(25);
                    }
                }
                if(thiTimer.milliseconds() > 400 && backDelay) {
                    backDelay = false;
                    robot.claw.setBig();
                    setPathState(25);
                }
                break;
            case 25:
                if(pathTimer.milliseconds()>600){
                    robot.clawBigTilt.setTransfer();
                    robot.depoWrist.setTransfer();
                    robot.claw.setOpen();
                }
                if(pathTimer.milliseconds()>950){
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
        //telemetry.addLine("Straight encoder: ");
        //telemetry.addLine("color: "+robot.sensorF.getColor());
        //telemetry.addLine("vLimit" + vlimitswitch.getState());
        //telemetry.addLine("hLimit" + hlimitswitch.getState());
        telemetry.addLine("Rep Count: " + floorRep);
        //telemetry.addLine("pathTimer: " + pathTimer);
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
                robot.vSlides.vSlidesL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.vSlides.vSlidesR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    }

    public static void waitFor(int milliseconds) { //Waitor Function
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < milliseconds) {
            // loop
        }
    }
}