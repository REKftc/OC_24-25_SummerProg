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
import overcharged.components.vSlides;
import overcharged.drive.SampleMecanumDrive;

import overcharged.pedroPathing.follower.Follower;
import overcharged.pedroPathing.localization.Pose;
import overcharged.pedroPathing.pathGeneration.BezierCurve;
import overcharged.pedroPathing.pathGeneration.BezierLine;
import overcharged.pedroPathing.pathGeneration.Path;
import overcharged.pedroPathing.pathGeneration.PathChain;
import overcharged.pedroPathing.pathGeneration.Point;
import overcharged.pedroPathing.util.Timer;

// Main Class
@Autonomous(name = "0+4 bucket now", group = "1Autonomous")
public class autoBucket4_135 extends OpMode{

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

    long totalTime;


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

    // LOCATIONS
    // GUIDE:
    // (0,0) is the corner. (144, 144) is the opposite.

    // OTHER POSES
    private Pose initBucket, beforeBucket, ready2Score, wallScore, beforeBucket2, beforeBucket3;
    private Pose startPose = new Pose(136, 32, Math.toRadians(90));

    private Path firstScore, inchBucket, goSafe, goBack, secondBack, floor2;

    private PathChain preload, floorCycle, toSub;

    private Follower follower;

    //TODO: Starting from here are the poses for the paths
    public void firstBucket(){
        beforeBucket = new Pose(120,19.75, Math.PI);
        beforeBucket2 = new Pose(120,13, Math.PI);
        ready2Score = new Pose(129.5,13,Math.toRadians(135));
        wallScore = new Pose(125.75,9.875, Math.PI);
    }

    //TODO: here are where the paths are defined
    public void buildPaths() {

        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose),new Point(ready2Score)))
                .setLinearHeadingInterpolation(startPose.getHeading(), ready2Score.getHeading())
                .setZeroPowerAccelerationMultiplier(3.5)
                .setPathEndTValueConstraint(0.7)
                .setPathEndVelocityConstraint(3)
                .build();

        firstScore = new Path(new BezierLine(new Point(startPose),new Point(ready2Score)));
        firstScore.setConstantHeadingInterpolation(ready2Score.getHeading());


        goSafe = new Path(new BezierLine(new Point(ready2Score), new Point(beforeBucket)));
        goSafe.setConstantHeadingInterpolation(Math.PI);

        secondBack = new Path(new BezierLine(new Point(wallScore), new Point(beforeBucket2)));
        secondBack.setConstantHeadingInterpolation(Math.PI);

        floor2 = new Path(new BezierLine(new Point(beforeBucket2), new Point(wallScore)));
        floor2.setConstantHeadingInterpolation(Math.PI);
        floor2.setZeroPowerAccelerationMultiplier(3);
        floor2.setPathEndTValueConstraint(0.95);
        floor2.setPathEndVelocityConstraint(5);

        floorCycle = follower.pathBuilder()
                .addPath(new BezierLine(new Point(beforeBucket), new Point(wallScore)))
                .setConstantHeadingInterpolation(Math.PI)
                .setZeroPowerAccelerationMultiplier(3)
                .setPathEndTValueConstraint(0.95)
                .setPathEndVelocityConstraint(5)
                .build();

        toSub = follower.pathBuilder()
                .addPath(new BezierCurve(
                                new Point(125.600, 9.000, Point.CARTESIAN),
                                new Point(125.000, 30.000, Point.CARTESIAN),
                                new Point(75.000, 10.000, Point.CARTESIAN),
                                new Point(82.000, 46.500, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .setZeroPowerAccelerationMultiplier(3)
                .setPathEndVelocityConstraint(3)
                .build();
    }


    // TODO: HERE IS WHERE THE MAIN PATH IS
    // Main pathing
    public void autoPath() {
        switch (pathState) {
            // Auto Body
            //
            case 10: // scores initial specimen
                robot.intakeTilt.setFlat();
                robot.vSlides.moveEncoderTo(robot.vSlides.high1, 1f);
                follower.followPath(preload, true);
                firstScore.setLinearHeadingInterpolation(startPose.getHeading(), ready2Score.getHeading());
                setPathState(12);
                nowDelay = false;
                break;
            case 12:
                if(pathTimer.milliseconds()>550){
                    pathTimer.reset();
                    robot.clawSmallTilt.setOut();
                    nowDelay = true;
                if(delayTimer.milliseconds()>300 && nowDelay){
                    nowDelay = false;
                    delayTimer.reset();
                    robot.depoWrist.setOut();
                    setPathState(13);
                    runOnce = true;
                    }
                }
                break;
            case 13:
                if(pathTimer.milliseconds()>550) {
                    if (runOnce) {
                        runOnce = false;
                        robot.clawBigTilt.setBucket();
                        robot.clawSmallTilt.setLeft();
                        robot.depoHslide.setInit();
                        delayTimer.reset();
                        nowDelay = true;
                    }
                    if (delayTimer.milliseconds() > 700 && nowDelay) {
                        nowDelay = false;
                        robot.claw.setBig();
                        scored = true;
                        runOnce = true;
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
                    if (doOnce){
                        doOnce = false;
                        floorRep -= 1;
                    }
                }
                waitFor(300);
                robot.depoWrist.setIn();
                nowDelay = true;
                if (nowDelay) {
                    waitFor(300);
                    nowDelay = false;
                    robot.claw.setOpen();
                    robot.clawBigTilt.setTransfer();
                    robot.clawSmallTilt.setTransfer();
                    if (scored) {
                        scored = false;
                        if (floorRep == 3) {
                            robot.latch.setOut();
                            robot.hslides.moveEncoderTo(robot.hslides.PRESET1, 1f);
                            follower.followPath(goSafe, false);
                            goSafe.setLinearHeadingInterpolation(ready2Score.getHeading(), Math.toRadians(180));
                            vslideGoBottom = true;
                            setPathState(16);
                        } else if (floorRep == 2) {
                            robot.latch.setOut();
                            robot.hslides.moveEncoderTo(robot.hslides.PRESET1, 1f);
                            follower.followPath(secondBack, false);
                            secondBack.setLinearHeadingInterpolation(wallScore.getHeading(), Math.toRadians(180));
                            vslideGoBottom = true;
                            setPathState(16);
                        } else if (floorRep == 1) {
                            robot.latch.setOut();
                            robot.hslides.moveEncoderTo(robot.hslides.PRESET2, 1f);
                            follower.followPath(secondBack, false);
                            secondBack.setLinearHeadingInterpolation(wallScore.getHeading(), Math.toRadians(205));
                            vslideGoBottom = true;
                            setPathState(16);
                        } else {
                            telemetry.addLine("aw man");
                        }
                    }
                }
                break;
            case 16:
                robot.intakeTilt.setOut();
                robot.latch.setOut();
                if(!follower.isBusy()) {
                    if (broken){
                        broken = false;
                        if (floorRep > 1){
                            robot.hslides.moveEncoderTo(robot.hslides.PRESET1, 0.8f);
                        }
                        else if (floorRep == 1){
                            robot.hslides.moveEncoderTo(robot.hslides.PRESET2, 0.8f);
                        }
                    }
                    else{
                        robot.hslides.moveEncoderTo(robot.hslides.hslides.getCurrentPosition()+145,0.7f);
                    }
                    in = false;
                    robot.intake.in();
                    setPathState(161);
                    runOnce = true;
                }
                break;
            case 161:
                if(robot.sensorF.getColor() == colorSensor.Color.YELLOW){
                    if (runOnce){
                        secTimer.reset();
                        runOnce = false;
                        robot.intakeTilt.setHigh();
                        hSlideGoBottom = true;
                        robot.intake.in();
                    }
                    waitFor(200);
                    secTimer.reset();
                    robot.intake.out();
                    setPathState(17);
                    nowDelay = false;
                    runOnce = true;
                }
                else {
                    if (pathTimer.milliseconds()>2100) {
                        pathTimer.reset();
                        robot.intake.off();
                        robot.intakeTilt.setTransfer();
                        hSlideGoBottom = true;
                        scored = true;
                        broken = true;
                        runOnce = true;
                        doOnce = true;
                        setPathState(14);
                    }
                }
                break;
            case 17:
                waitFor(250);
                robot.intake.off();
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
                            floorCycle.getPath(0).setLinearHeadingInterpolation(Math.toRadians(205), Math.PI);
                        }
                        backDelay = true;
                        doOnce = true;
                    }
                    if(secTimer.milliseconds()>200 && backDelay) {
                        if (doOnce) {
                            doOnce = false;
                            robot.claw.setClose();
                            nowDelay = true;
                            runOnce = true;
                        }
                    }
                    if (pathTimer.milliseconds()>250 && nowDelay) {
                        if (runOnce) {
                            runOnce = false;
                            nowDelay = false;
                            robot.intakeTilt.setFlat();
                            robot.vSlides.moveEncoderTo(robot.vSlides.high1, 1f);
                            s2Delay = true;
                            thiTimer.reset();
                        }
                    }
                    if (thiTimer.milliseconds()>200 && s2Delay) {
                        s2Delay = false;
                        robot.depoWrist.setOut();
                        setPathState(171);
                    }
                }

                break;
            case 171:
                if(pathTimer.milliseconds()>650){
                    pathTimer.reset();
                    robot.clawBigTilt.setBucket();
                    robot.depoHslide.setInit();
                    robot.clawSmallTilt.setRight();
                    robot.intakeTilt.setTransfer();
                    setPathState(1710);
                }
                break;
            case 1710:
                if(pathTimer.milliseconds() > 360){
                    if(floorRep==3){
                        tempWait = 620;
                    }
                    else{
                        tempWait = 495;
                    }
                    waitFor(tempWait);
                    delayTimer.reset();
                    robot.claw.setBig();
                    scored = true;
                    setPathState(172);

                }
                break;
            case 172:
                if (floorRep > 1) {
                    floorRep -= 1;
                    waitFor(300);
                    setPathState(14);
                }
                else{
                    setPathState(18);
                    runOnce = true;
                }
                break;
            case 18:
                if(delayTimer.milliseconds()>400) {
                    if (runOnce) {
                        runOnce = false;
                        delayTimer.reset();
                        robot.depoWrist.setIn();
                        nowDelay = true;
                        secTimer.reset();
                    }
                }
                if (secTimer.milliseconds()>500 && nowDelay) {
                    secTimer.reset();
                    nowDelay = false;
                    robot.claw.setOpen();
                    robot.clawBigTilt.setTransfer();
                    vslideGoBottom = true;
                    robot.clawSmallTilt.setTransfer();
                    setPathState(19);
                }
                break;
            case 19:
                if(!follower.isBusy()){
                    follower.followPath(toSub);
                    setPathState(191);
                }
                break;
            case 191:
                if(pathTimer.milliseconds()>450) {
                    pathTimer.reset();
                    robot.intakeTilt.setTransfer();
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
        telemetry.addLine("Path: " + pathState);
        telemetry.addLine("Position: " + follower.getPose());
        telemetry.addLine("heading: " + follower.getTotalHeading());
        //telemetry.addLine("color: "+robot.sensorF.getColor());
        //telemetry.addLine("vLimit" + vlimitswitch.getState());
        //telemetry.addLine("hLimit" + hlimitswitch.getState());
        telemetry.addLine("Rep Count"+ floorRep);
        telemetry.addLine("pathTimer: " + pathTimer);
        telemetry.addLine("delayTimer: " + delayTimer);
        telemetry.addLine("secTimer: " + secTimer);
        telemetry.addLine("thiTimer: " + thiTimer);

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