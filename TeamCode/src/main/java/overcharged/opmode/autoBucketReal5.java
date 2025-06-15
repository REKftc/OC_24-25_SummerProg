package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
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

@Autonomous(name = "5 bucket", group = "0Autonomous")
public class autoBucketReal5 extends OpMode{

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
    boolean brokeCheck = false;
    int turnExtra = 0;

    boolean runOnce;

    long tempTime;

    private int freeBlocks = 1;

    //TODO: poses
    private Pose startPose = new Pose(134, 25, Math.toRadians(135));
    private Point bucketScore, toSub;
    private Pose onSub;

    private Path hi;

    private PathChain initScore, subCycle, subToBucket;

    private CustomPIDFCoefficients strongHead = new CustomPIDFCoefficients(2,0,0.0,0);
    private CustomPIDFCoefficients normHead = new CustomPIDFCoefficients(0.95,0,0.0,0);
    private CustomFilteredPIDFCoefficients weakDrive = new CustomFilteredPIDFCoefficients(1.5, 0.000001,0,0.6, 0);
    private CustomFilteredPIDFCoefficients normDrive = new CustomFilteredPIDFCoefficients(1, 0.000001,0,0.6, 0);

    public void buildPoses() {

        bucketScore = new Point(129, 12, Point.CARTESIAN);

        toSub = new Point(90, 25, Point.CARTESIAN);
        onSub = new Pose(84, 49, Math.toRadians(90));
    }

    //TODO: here are where the paths are defined
    public void buildPaths() {
        initScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), bucketScore))
                //.addPath(new BezierCurve(new Point(startPose), new Point(138, 10), bucketScore))
                .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(155))
                .build();

        subCycle = follower.pathBuilder()
                .addPath(new BezierCurve(bucketScore, toSub, new Point(onSub)))
                .setLinearHeadingInterpolation(Math.toRadians(155), onSub.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();

        subToBucket = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(onSub), toSub, bucketScore))
                .setLinearHeadingInterpolation(onSub.getHeading(), Math.toRadians(155))
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();


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
                    follower.setHeadingPIDF(normHead);
                    robot.vSlides.moveEncoderTo(robot.vSlides.high1, 1f);
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
                    setPathState(12);
                }
                break;
            case 12:
                if(follower.getCurrentTValue() > 0.7){
                    //follower.setHeadingPIDF(strongHead);
                    setPathState(13);
                }
                break;
            case 13:
                if ((follower.getPose().getX() > (bucketScore.getX() - 1) && follower.getPose().getY() < (bucketScore.getY() + 1) && Math.abs(robot.vSlides.vSlidesL.getCurrentPosition()-660) < 16) && pathTimer.milliseconds()>800 || pathTimer.milliseconds()>1100 || pathTimer.milliseconds() > 380 && turnExtra >0) {
                    setPathState(14);
                }
                break;
            case 14:
                if(pathTimer.milliseconds()>180) {
                    robot.claw.setOpen();
                    robot.intakeTilt.setOut();
                    robot.intake.in();
                    if (turnExtra < 3) {
                        setPathState(15);
                    } else {
                        setPathState(18);
                    }
                }
                break;
            case 15:
                if(pathTimer.milliseconds()>250){
                    //follower.setHeadingPIDF(normHead);
                    robot.depoTilt.setTransfer();
                    robot.depoHslide.setTransfer();
                    if(turnExtra > 0) {
                        follower.setHeadingPIDF(strongHead);
                        if (turnExtra == 1) {
                            follower.turnDegrees(22, true);
                        } else if (turnExtra == 2) {
                            if(!brokeCheck) {
                                follower.turnDegrees(50, true);
                            } else{
                                follower.turnDegrees(28, true);
                            }
                        }
                    }
                    setPathState(16);
                }
                break;
            case 16:
                if(pathTimer.milliseconds() > 180) {
                    vslideGoBottom = true;
                    in = false;
                    if (turnExtra == 0) {
                        robot.hslides.moveEncoderTo(robot.hslides.PRESET3, 0.95f);
                    } else if (turnExtra == 1){
                        robot.hslides.moveEncoderTo(robot.hslides.PRESET2, 0.95f);
                    } else if (turnExtra == 2){
                        robot.hslides.moveEncoderTo(robot.hslides.PRESET1, 0.95f);
                    }
                    setPathState(161);
                    runOnce = true;
                }
                break;
            case 161:
                if(pathTimer.milliseconds() > 200){
                    follower.setHeadingPIDF(normHead);
                    setPathState(17);
                }
                break;
            case 17:
                if(robot.sensorF.getColor() == colorSensor.Color.YELLOW){
                    setPathState(171);
                } else if(robot.sensorF.getColor() == colorSensor.Color.NONE && pathTimer.milliseconds()<1500 && robot.hslides.hslides.getCurrentPosition() < 700) {
                    robot.hslides.moveEncoderTo(robot.hslides.hslides.getCurrentPosition() + 100, 1f);
                } else if(robot.sensorF.getColor() == colorSensor.Color.NONE && pathTimer.milliseconds()>1700 || robot.hslides.hslides.getCurrentPosition() > 700)  {
                    setPathState(1701);
                }

                break;
            case 1701:
                robot.intakeTilt.setFlat();
                robot.hslides.moveEncoderTo(hslides.FAIL, 1f);
                setPathState(1702);
                break;
            case 1702:
                if(pathTimer.milliseconds()>200){
                    turnExtra += 1;
                    brokeCheck = true;
                    setPathState(14);
                }
                break;
            case 171:
                if (runOnce){
                    runOnce = false;
                    robot.intakeTilt.setTransfer();
                    robot.intake.in();
                    hSlideGoBottom = true;
                }
                if(pathTimer.milliseconds() > 180) {
                    if(turnExtra > 0) {
                        follower.setHeadingPIDF(strongHead);
                        if (turnExtra == 1) {
                            follower.turnDegrees(22, false);
                        } else if (turnExtra == 2) {
                            follower.turnDegrees(50, false);
                        }
                    }
                    setPathState(1711);
                    tempTime = 0;

                }
                break;
            case 1711:
                if(in && pathTimer.milliseconds()>90){
                    setPathState(1712);
                }
                break;
            case 1712:
                if(pathTimer.milliseconds()>150){
                    robot.claw.setClose();
                    if (turnExtra<3) {
                        turnExtra += 1;
                        setPathState(102);
                    } else{
                        setPathState(18);
                    }
                }
                break;
            case 18:
                if(pathTimer.milliseconds()>200) {
                    robot.intakeTilt.setTransfer();
                    follower.setDrivePIDF(weakDrive);
                    follower.followPath(subCycle, true);
                    robot.depoTilt.setTransfer();
                    robot.depoHslide.setTransfer();
                    setPathState(19);
                }
                break;
            case 19:
                if(pathTimer.milliseconds() > 150){
                    vslideGoBottom = true;
                    robot.latch.setOut();
                    setPathState(20);
                }
                break;
            case 20:
                if(follower.getCurrentTValue() > 0.8){
                    robot.hslides.moveEncoderTo(hslides.SMALL, 1f);
                    setPathState(201);
                }
                break;
            case 201:
                if(follower.getCurrentTValue() > 0.95 || follower.getCurrentTValue() == 0.0) {
                    robot.intakeTilt.setOut();
                    setPathState(21);
                }
                break;
            case 21:
                if(robot.sensorF.getColor() == colorSensor.Color.YELLOW || robot.sensorF.getColor() == colorSensor.Color.RED){
                    setPathState(22);
                } else if(robot.sensorF.getColor() == colorSensor.Color.NONE && pathTimer.milliseconds()<1900 && robot.hslides.hslides.getCurrentPosition() < 700) {
                    robot.hslides.moveEncoderTo(robot.hslides.hslides.getCurrentPosition() + 70, 0.9f);
                } else if(robot.sensorF.getColor() == colorSensor.Color.NONE && pathTimer.milliseconds()>2000 || robot.hslides.hslides.getCurrentPosition() > 700) {
                    setPathState(211);
                } else if (robot.sensorF.getColor() == colorSensor.Color.BLUE){
                    robot.intake.out();
                    waitFor(350);
                    robot.intake.in();
                } else {
                    brokeCheck = true;
                    setPathState(211);
                }
                break;
            case 211:
                robot.intakeTilt.setFlat();
                hSlideGoBottom = true;
                in = false;
                follower.setHeadingPIDF(strongHead);
                follower.turnDegrees(10, true);
                setPathState(212);
                break;
            case 212:
                if(in){
                    robot.hslides.moveEncoderTo(hslides.SMALL, 1f);
                    setPathState(2121);
                }
                break;
            case 2121:
                if(pathTimer.milliseconds()>300) {
                    setPathState(201);
                }
                break;
            case 2101: //TODO: wtf
                if (pathTimer.milliseconds() > 400){
                    robot.intake.in();
                    setPathState(21);
                }
                break;
            case 22:
                robot.intakeTilt.setHigh();
                hSlideGoBottom = true;
                in = false;
                follower.setHeadingPIDF(normHead);
                follower.setDrivePIDF(weakDrive);
                setPathState(23);
                break;
            case 23:
                follower.followPath(subToBucket, true);
                setPathState(231);
                break;
            case 231:
                if(pathTimer.milliseconds()>200){
                    robot.intake.slowOut();
                    setPathState(232);
                }
                break;
            case 232:
                if(pathTimer.milliseconds()>200){
                    robot.intake.in();
                    setPathState(24);
                }
                break;
            case 24:
                if(follower.getCurrentTValue() > 0.2 && in){
                    robot.intakeTilt.setTransfer();
                    setPathState(2401);
                }
                break;
            case 2401:
                if (pathTimer.milliseconds() > 100){
                    robot.claw.setClose();
                    setPathState(2402);
                }
                break;
            case 2402:
                if(follower.getCurrentTValue() > 0.4 && pathTimer.milliseconds()>150){
                    robot.vSlides.moveEncoderTo(vSlides.high1, 1f);
                    setPathState(2403);
                }
                break;
            case 2403:
                if(pathTimer.milliseconds()>150) {
                    robot.depoTilt.setOut();
                    robot.depoHslide.setOut();
                    setPathState(241);
                }
                break;
            case 241:
                if(follower.getCurrentTValue() > 0.9 && follower.getPose().getX() > (bucketScore.getX() - 1) && follower.getPose().getY() < (bucketScore.getY() + 1) && Math.abs(robot.vSlides.vSlidesL.getCurrentPosition()-660) < 16 && pathTimer.milliseconds()>400){
                    setPathState(25);
                }
                break;
            case 25:
                if(pathTimer.milliseconds() > 250){
                    robot.claw.setOpen();
                    //follower.setDrivePIDF(normDrive);
                    setPathState(18);
                }


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
        robot.vSlides.update();
        autoPath();
        telemetry.addLine("turnExtra: " + turnExtra);
        telemetry.addLine("TValue: " + follower.getCurrentTValue());
        telemetry.addLine("Case: " + pathState);
        telemetry.addLine("Position: " + follower.getPose());
        telemetry.addLine("heading: " + follower.getTotalHeading());
        telemetry.addData("lag: ", temp);
        telemetry.addLine("color: "+robot.sensorF.getColor());
        telemetry.addLine("broken?: "+brokeCheck);

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
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        //Pose init
        buildPoses();
        buildPaths();

        //robot init
        hlimitswitch = hardwareMap.get(DigitalChannel.class, "hlimitswitch");
        vlimitswitch = hardwareMap.get(DigitalChannel.class, "vlimitswitch");
        robot.vSlides.vSlidesR.resetPosition();
        robot.vSlides.vSlidesL.resetPosition();
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

    public static void waitFor(int milliseconds) { //Waitor Function
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < milliseconds) {
            // loop
        }
    }
}
