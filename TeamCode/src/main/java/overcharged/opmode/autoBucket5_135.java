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

import overcharged.pedroPathing.follower.Follower;
import overcharged.pedroPathing.localization.Pose;
import overcharged.pedroPathing.pathGeneration.BezierCurve;
import overcharged.pedroPathing.pathGeneration.BezierLine;
import overcharged.pedroPathing.pathGeneration.Path;
import overcharged.pedroPathing.pathGeneration.PathChain;
import overcharged.pedroPathing.pathGeneration.Point;

// Main Class
@Autonomous(name = "0+5 red bucket test", group = "1Autonomous")
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
        beforeBucket = new Pose(120,26.35, Math.PI);
        beforeBucket2 = new Pose(120,18, Math.PI);
        ready2Score = new Pose(126.15,19.75,Math.toRadians(135));
        wallScore = new Pose(124.15,14.25, Math.PI);
        iShouldGetCloserToWallBecauseMyRobotKeepsOnDying = new Pose(125.40,13.55, Math.PI);
        subFront = new Pose(83.000, 52.000, Math.PI/2);
    }

    //TODO: here are where the paths are defined
    public void buildPaths() {

        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose),new Point(ready2Score)))
                .setLinearHeadingInterpolation(startPose.getHeading(), ready2Score.getHeading())
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
                follower.setMaxPower(0.8f);
                setPathState(12);
                nowDelay = false;
                break;
            case 12:
                if(pathTimer.milliseconds()>150){
                    nowDelay = true;
                    if(delayTimer.milliseconds()>450 && nowDelay){
                        nowDelay = false;
                        robot.latch.setOut();
                        robot.hslides.moveEncoderTo(robot.hslides.SMALL, 1f);
                        setPathState(13);
                        runOnce = true;
                    }
                }
                break;
            case 13:
                if(pathTimer.milliseconds()>200 && robot.vSlides.vSlidesL.getCurrentPosition()>500) {
                    if (runOnce) {
                        runOnce = false;
                        robot.clawSmallTilt.setOut();
                        robot.clawBigTilt.setBucket();
                        robot.depoWrist.setBucket();
                        robot.depoHslide.setInit();
                        delayTimer.reset();
                        nowDelay = true;
                    }
                    if (delayTimer.milliseconds() > 280 && nowDelay){
                        canScore = true;
                        doOnce = true;
                    }
                    if (follower.getPose().getX() > (ready2Score.getX() - 1) && follower.getPose().getY() > (ready2Score.getY() - 1.4) && Math.abs(robot.vSlides.vSlidesL.getCurrentPosition()-805) < 30 && canScore) {
                        if (doOnce) {
                            doOnce = false;
                            thiTimer.reset();
                            nowDelay = false;
                            backDelay = true;
                            canScore = false;
                        }
                    }
                    if (thiTimer.milliseconds() > 350 && backDelay){
                        backDelay = false;
                        robot.claw.setBig();
                        runOnce = true;
                        canScore = false;
                        setPathState(14);
                    }
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
                    robot.hslides.moveEncoderTo(robot.hslides.PRESET1, 1f);
                    if (doOnce){
                        doOnce = false;
                        floorRep -= 1;
                    }
                }
                waitFor(300);
                robot.clawBigTilt.setTransfer();
                robot.depoWrist.setTransfer();
                robot.clawSmallTilt.setTransfer();
                waitFor(150);
                robot.claw.setOpen();
                nowDelay = false;
                scored=true;
                if (scored) {
                    scored = false;
                    runOnce = true;
                    if (floorRep == 3) {
                        follower.followPath(goSafe, true);
                        goSafe.setLinearHeadingInterpolation(ready2Score.getHeading(), Math.toRadians(180));
                        setPathState(16);
                    } else if (floorRep == 2) {
                        follower.followPath(secondBack, true);
                        setPathState(16);
                    } else if (floorRep == 1) {
                        follower.followPath(secondBack, true);
                        secondBack.setLinearHeadingInterpolation(wallScore.getHeading(), Math.toRadians(200));
                        setPathState(16);
                    } else {
                        telemetry.addLine("aw man");
                    }
                }

                break;
            case 16:
                robot.intake.in();
                robot.intakeTilt.setInOut();
                if(pathTimer.milliseconds()>450){
                    if (runOnce) {
                        runOnce = false;
                        vslideGoBottom = true;
                    }
                }
                if(!follower.isBusy()) {
                    in = false;
                    runOnce = true;
                    robot.depoWrist.setTransfer();
                    robot.clawBigTilt.setTransfer();
                    robot.clawSmallTilt.setTransfer();
                    robot.intakeTilt.setOut();
                    setPathState(161);
                }
                break;
            case 161:
                if(robot.sensorF.getColor() == colorSensor.Color.YELLOW){
                    tempTime = System.currentTimeMillis();
                    if (runOnce){
                        secTimer.reset();
                        runOnce = false;
                        robot.depoWrist.setTransfer();
                        robot.clawBigTilt.setTransfer();
                        robot.clawSmallTilt.setTransfer();
                        robot.intakeTilt.setTransfer();
                        //hSlideGoBottom = true;
                        robot.intake.in();
                    }
                    if(tempTime > 500) {
                        hSlideGoBottom = true;
                        setPathState(17);
                        nowDelay = false;
                        runOnce = true;

                    }
                }
                else if(robot.sensorF.getColor() == colorSensor.Color.NONE && pathTimer.milliseconds()<1800) {
                    if (broken){
                        broken = false;
                        if (floorRep == 3){
                            robot.latch.setOut();
                            robot.hslides.moveEncoderTo(robot.hslides.PRESET1+80, 0.7f);
                        }
                        else if (floorRep == 2){
                            robot.latch.setOut();
                            robot.hslides.moveEncoderTo(robot.hslides.PRESET1+10, 0.9f);
                        }
                        else if (floorRep == 1){
                            robot.latch.setOut();
                            robot.hslides.moveEncoderTo(robot.hslides.PRESET2+40, 0.8f);
                        }
                    }
                    else{
                        if (floorRep ==3){
                            robot.hslides.moveEncoderTo(robot.hslides.hslides.getCurrentPosition() + 140, 0.7f);
                        }
                        else if (floorRep == 2){
                            robot.hslides.moveEncoderTo(robot.hslides.hslides.getCurrentPosition() + 140, 0.9f);
                        }
                        else {
                            robot.hslides.moveEncoderTo(robot.hslides.hslides.getCurrentPosition() + 140, 0.8f);
                        }
                    }
                }
                if (robot.sensorF.getColor() == colorSensor.Color.NONE && pathTimer.milliseconds()>1810) {
                    pathTimer.reset();
                    robot.intake.out();
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
                        robot.intakeTilt.setTransfer();
                        if (floorRep == 3) {
                            follower.followPath(floorCycle, true);
                        } else if (floorRep >0) {
                            follower.followPath(floor2, true);
                        }
                        if (floorRep == 1) {
                            floor2.setLinearHeadingInterpolation(Math.toRadians(200), Math.PI);
                        }

                        backDelay = true;
                        //doOnce = true;
                    }
                    //delay claw close once hslides are back
                    if(!hSlideGoBottom && backDelay){
                        tempTime = System.currentTimeMillis();
                        doOnce = true;
                        backDelay = false;
                    }
                    //claw close delay
                    if(tempTime>200 && doOnce) {
                            doOnce = false;
                            robot.claw.setClose();
                            robot.intake.off();
                            nowDelay = true;
                            runOnce = true;
                            tempTime = System.currentTimeMillis();
                            //delayTimer.reset();
                    }
                    //vslide up delay
                    if (tempTime>300 && nowDelay) {
                        if (runOnce) {
                            runOnce = false;
                            nowDelay = false;
                            robot.intakeTilt.setFlat();
                            robot.vSlides.moveEncoderTo(robot.vSlides.autohigh1, 1f);
                            s2Delay = true;
                        }
                    }
                    if (s2Delay && robot.vSlides.vSlidesL.getCurrentPosition()>500) {
                        s2Delay = false;
                        robot.clawBigTilt.setBucket();
                        robot.depoWrist.setBucket();
                        setPathState(171);
                    }
                }

                break;
            case 171:
                if(pathTimer.milliseconds()>250){
                    robot.depoHslide.setInit();
                    robot.clawSmallTilt.setRight();
                    robot.intakeTilt.setTransfer();
                    if(floorRep>1) {
                        robot.latch.setOut();
                        robot.hslides.moveEncoderTo(robot.hslides.PRESET1, 1f);
                    } else if (floorRep == 1){
                        robot.latch.setOut();
                        robot.hslides.moveEncoderTo(robot.hslides.PRESET2, 1f);
                    }
                    setPathState(1710);
                }
                break;
            case 1710:
                if(pathTimer.milliseconds() > 200 && follower.getPose().getX() > (wallScore.getX() - 1.2) && follower.getPose().getY() > (wallScore.getY() - 1.8) && Math.abs(robot.vSlides.vSlidesL.getCurrentPosition()-805) < 24){
                    robot.claw.setBig();
                    scored = true;
                    setPathState(172);
                }
                break;
            case 172:
                if(pathTimer.milliseconds() > 100) {
                    if (floorRep > 1) {
                        floorRep -= 1;
                        setPathState(14);
                    } else {
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
                if (secTimer.milliseconds()>150 && nowDelay) {
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
                        follower.followPath(slightMove, false);
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
                    }
                }
                else{
                    moveCoEff += 1;
                    robot.trapdoor.setInit();
                    setPathState(211);
                }
                if(robot.sensorF.getColor() == colorSensor.Color.RED || robot.sensorF.getColor() == colorSensor.Color.YELLOW){
                    robot.trapdoor.setInit();
                    robot.intakeTilt.setTransfer();
                    waitFor(300);
                    robot.intake.out();
                    setPathState(22);
                }
                else if (robot.sensorF.getColor() == colorSensor.Color.BLUE){
                    robot.trapdoor.setOut();
                    robot.intake.off();
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
                if(pathTimer.milliseconds()>190) {
                    robot.intake.out();
                    hSlideGoBottom = true;
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
                    setPathState(24);
                }
                break;
            case 24:
                if(pathTimer.milliseconds()>500 && nowDelay && robot.hslides.hslides.getCurrentPosition() < 50){
                    robot.intake.off();
                    robot.claw.setClose();
                    nowDelay =false;
                    runOnce =true;
                }
                if(pathTimer.milliseconds()>680){
                    if(runOnce) {
                        runOnce = false;
                        s2Delay = true;
                        robot.vSlides.moveEncoderTo(robot.vSlides.autohigh1, 1f);
                        thiTimer.reset();
                    }
                }
                if(pathTimer.milliseconds()>780 && s2Delay) {
                    robot.intakeTilt.setFlat();
                }
                if (thiTimer.milliseconds()>300 && s2Delay) {
                    s2Delay = false;
                    setPathState(241);
                }
                break;
            case 241:
                if(follower.getPose().getX() > (wallScore.getX() - 12) && follower.getPose().getY() < (wallScore.getY() + 12)){
                    robot.clawBigTilt.setBucket();
                    robot.depoWrist.setBucket();
                    runningOutOfNames = true;
                }
                if(pathTimer.milliseconds()>200 && robot.vSlides.vSlidesL.getCurrentPosition() > 500 && runningOutOfNames){
                    robot.depoHslide.setInit();
                    robot.clawSmallTilt.setRight();
                    robot.intakeTilt.setTransfer();
                    canScore = true;
                    runOnce = true;
                }
                if(pathTimer.milliseconds()>200 && canScore && follower.getPose().getX() > (wallScore.getX() - 1.2) && follower.getPose().getY() > (wallScore.getY() - 1.8) && Math.abs(robot.vSlides.vSlidesL.getCurrentPosition()-805) < 25){
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
                if(pathTimer.milliseconds()>800){
                    robot.intakeTilt.setTransfer();
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
        telemetry.addLine("TValue: "+follower.getCurrentTValue());
        telemetry.addLine("Case: " + pathState);
        telemetry.addLine("Position: " + follower.getPose());
        telemetry.addLine("heading: " + follower.getTotalHeading());
        //telemetry.addLine("Straight encoder: ");
        //telemetry.addLine("color: "+robot.sensorF.getColor());
        //telemetry.addLine("vLimit" + vlimitswitch.getState());
        //telemetry.addLine("hLimit" + hlimitswitch.getState());
        telemetry.addLine("Rep Count: "+ floorRep);
        //telemetry.addLine("pathTimer: " + pathTimer);
        //telemetry.addLine("delayTimer: " + delayTimer);
        //telemetry.addLine("secTimer: " + secTimer);
        //telemetry.addLine("thiTimer: " + thiTimer);
        telemetry.addLine("delayTimer: " + tempTime);
        telemetry.addLine("doOnce: " + doOnce);
        telemetry.addLine("backDelay: " + backDelay);
        telemetry.addLine("s2Delay: " + s2Delay);
        telemetry.addLine("nowDelay: " + nowDelay);
        telemetry.addLine("hslideGoBottom: " + hSlideGoBottom);
        telemetry.addLine("vslides: " + robot.vSlides.vSlidesR.getCurrentPosition());
        telemetry.addLine("color: " + robot.sensorF.getColor());
        //functions
        if (!hlimitswitch.getState() && hSlideGoBottom) {
            in = true;
            robot.latch.setInit();
            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.hslides.hslides.setPower(-1);
            RobotLog.ii(TAG_SL, "Going down");
        } else if (hlimitswitch.getState() && hSlideGoBottom) {
            robot.latch.setInit();
            robot.intakeTilt.setTransfer();
            robot.hslides.hslides.setPower(0);
            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.hslides.hslides.resetPosition();

            hSlideGoBottom = false;
            RobotLog.ii(TAG_SL, "Force stopped");
        }

        if (!vlimitswitch.getState() && vslideGoBottom) {
            robot.vSlides.vSlidesL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.vSlides.vSlidesR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.vSlides.vSlidesR.setPower(-0.7f);
            robot.vSlides.vSlidesL.setPower(-0.7f);
            RobotLog.ii(TAG_SL, "Going down");
        } else if (vlimitswitch.getState() && vslideGoBottom) {
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
        delayTimer = new ElapsedTime();
        pathTimer = new ElapsedTime();
        secTimer = new ElapsedTime();
        thiTimer = new ElapsedTime();


        //follower init
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