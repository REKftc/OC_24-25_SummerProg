package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.pathgen.BezierPoint;
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
import com.pedropathing.util.Timer;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import overcharged.pedroPathing.constants.FConstants;
import overcharged.pedroPathing.constants.LConstants;

@Autonomous(name = "red spec farther", group = "0Autonomous")
public class autoRedSpecimenNew extends OpMode {

    // Init
    private RobotMecanum robot;
    private DigitalChannel vlimitswitch;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SampleMecanumDrive drive;
    MultipleTelemetry telems;
    private Timer pathTimer, opmodeTimer;

    private int pathState;

    private Follower follower;

    boolean vslideGoBottom = false;

    //TODO: poses
    private Pose startPose = new Pose(135, 64, Math.PI);
    private Pose beforeSpecimen, curveControlPoint, curveAnControlPoint, curveEndPoint, goBack1, curve2ControlPoint, curve2AnControlPoint, curve2EndPoint, goBack2, curve3ControlPoint, curve3AnControlPoint, curve3EndPoint, goBack3, getSpec2, getSpec2Back, scoreSpec2Mid, scoreSpec2End, scoreSpec3Mid, scoreSpec3End, scoreSpec4Mid, scoreSpec4End, scoreSpec5Mid, scoreSpec5End, park;

    private Path curve, curve2, curve3, curve4, curve5, curve6, curve7, curve8, curve9, curve10, curveLast, parkEnd;

    private PathChain preload;

    public void buildPoses() {
        beforeSpecimen = new Pose(107, 64, Math.PI);
        curveControlPoint = new Pose(120, 96, Math.PI);
        curveAnControlPoint = new Pose(85, 96, Math.PI);
        curveEndPoint = new Pose(55, 106, Math.PI);
        goBack1 = new Pose(126, 106, Math.PI);
        curve2ControlPoint = new Pose(120, 101, Math.PI);
        curve2AnControlPoint = new Pose(85, 101, Math.PI);
        curve2EndPoint = new Pose(55, 115, Math.PI);
        goBack2 = new Pose(126, 115, Math.PI);
        curve3ControlPoint = new Pose(120, 113, Math.PI);
        curve3AnControlPoint = new Pose(85, 113, Math.PI);
        curve3EndPoint = new Pose(55, 121, Math.PI);
        goBack3 = new Pose(126, 121, Math.PI);
        getSpec2 = new Pose(122, 90, Math.PI);
        getSpec2Back = new Pose(131, 90, Math.PI);
        scoreSpec2Mid = new Pose(131, 62, Math.PI);
        scoreSpec2End = new Pose(106, 62, Math.PI);
        scoreSpec3Mid = new Pose(131, 66, Math.PI);
        scoreSpec3End = new Pose(106, 66, Math.PI);
        scoreSpec4Mid = new Pose(131, 60, Math.PI);
        scoreSpec4End = new Pose(106, 60, Math.PI);
        scoreSpec5Mid = new Pose(131, 58, Math.PI);
        scoreSpec5End = new Pose(106, 58, Math.PI);
        park = new Pose(132, 105, Math.PI);
    }

    //TODO: here are where the paths are defined
    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(beforeSpecimen)))
                //.setConstantHeadingInterpolation(startPose.getHeading())
                .setLinearHeadingInterpolation(startPose.getHeading(), beforeSpecimen.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        curve = new Path(new BezierCurve(new Point(beforeSpecimen), new Point(curveControlPoint), new Point(curveAnControlPoint), new Point(curveEndPoint), new Point(goBack1)));
        curve.setLinearHeadingInterpolation(beforeSpecimen.getHeading(), goBack1.getHeading());
        curve.setPathEndTimeoutConstraint(0);

        curve2 = new Path(new BezierCurve(new Point(goBack1), new Point(curve2ControlPoint), new Point(curve2AnControlPoint), new Point(curve2EndPoint), new Point(goBack2)));
        curve2.setLinearHeadingInterpolation(goBack1.getHeading(), goBack2.getHeading());
        curve2.setPathEndTimeoutConstraint(0);

        curve3 = new Path(new BezierCurve(new Point(goBack2), new Point(curve3ControlPoint), new Point(curve3AnControlPoint), new Point(curve3EndPoint), new Point(goBack3)));
        curve3.setLinearHeadingInterpolation(goBack2.getHeading(), goBack3.getHeading());
        curve3.setPathEndTimeoutConstraint(0);

        curve4 = new Path(new BezierCurve(new Point(goBack3), new Point(getSpec2), new Point(getSpec2Back)));
        curve4.setLinearHeadingInterpolation(goBack2.getHeading(), getSpec2Back.getHeading());
        curve4.setPathEndTimeoutConstraint(0);

        curve5 = new Path(new BezierCurve(new Point(getSpec2Back), new Point(scoreSpec2Mid), new Point(scoreSpec2End)));
        curve5.setLinearHeadingInterpolation(getSpec2Back.getHeading(), scoreSpec2End.getHeading());
        curve5.setPathEndTimeoutConstraint(0);

        curve6 = new Path(new BezierCurve(new Point(scoreSpec2End), new Point(scoreSpec2Mid), new Point(getSpec2), new Point(getSpec2Back)));
        curve6.setLinearHeadingInterpolation(scoreSpec2End.getHeading(), getSpec2Back.getHeading());
        curve6.setPathEndTimeoutConstraint(0);

        curve7 = new Path(new BezierCurve(new Point(getSpec2Back), new Point(scoreSpec3Mid), new Point(scoreSpec3End)));
        curve7.setLinearHeadingInterpolation(getSpec2Back.getHeading(), scoreSpec3End.getHeading());
        curve7.setPathEndTimeoutConstraint(0);

        curve8 = new Path(new BezierCurve(new Point(scoreSpec3End), new Point(scoreSpec3Mid), new Point(getSpec2), new Point(getSpec2Back)));
        curve8.setLinearHeadingInterpolation(scoreSpec3End.getHeading(), getSpec2Back.getHeading());
        curve8.setPathEndTimeoutConstraint(0);

        curve9 = new Path(new BezierCurve(new Point(getSpec2Back), new Point(scoreSpec4Mid), new Point(scoreSpec4End)));
        curve9.setLinearHeadingInterpolation(getSpec2Back.getHeading(), scoreSpec4End.getHeading());
        curve9.setPathEndTimeoutConstraint(0);

        curve10 = new Path(new BezierCurve(new Point(scoreSpec4End), new Point(scoreSpec4Mid), new Point(getSpec2), new Point(getSpec2Back)));
        curve10.setLinearHeadingInterpolation(scoreSpec4End.getHeading(), getSpec2Back.getHeading());
        curve10.setPathEndTimeoutConstraint(0);

        curveLast = new Path(new BezierCurve(new Point(getSpec2Back), new Point(scoreSpec5Mid), new Point(scoreSpec5End)));
        curveLast.setLinearHeadingInterpolation(getSpec2Back.getHeading(), scoreSpec5End.getHeading());
        curveLast.setPathEndTimeoutConstraint(0);

        parkEnd = new Path(new BezierLine(new Point(scoreSpec5End), new Point(park)));
        parkEnd.setLinearHeadingInterpolation(scoreSpec5End.getHeading(), park.getHeading());
    }


    // TODO: HERE IS WHERE THE MAIN PATH IS
    // Main pathing
    public void autoPath() {
        switch (pathState) {
            case 1: // start
                follower.followPath(preload,true);
                follower.setMaxPower(0.8f);
                robot.vSlides.moveEncoderTo(robot.vSlides.mid-15, 1f);
                robot.depoHslide.setOut();
                setPathState(2);
                break;

            case 2:
                if(!follower.isBusy()) {
                    robot.claw.setOpen();
                    robot.depoHslide.setInit();
                    follower.followPath(curve);
                    follower.setMaxPower(0.9f);
                    vslideGoBottom = true;
                    robot.depoTilt.setWallAuto();

                    if(pathTimer.getElapsedTimeSeconds() > 5) {
                        telems.addLine("curve: 2");
                        follower.followPath(curve2);
                        follower.setMaxPower(0.9f);
                    }

                    if(pathTimer.getElapsedTimeSeconds() > 8) {
                        telems.addLine("curve: 3");
                        follower.followPath(curve3);
                        follower.setMaxPower(0.9f);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.8f);
                    follower.followPath(curve4);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    robot.claw.setClose();
                    waitFor(200);
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid, 1f);
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid, 1f);
                    robot.depoTilt.setSpec();
                    robot.depoHslide.setOut();
                    follower.setMaxPower(0.9f);
                    follower.followPath(curve5);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    robot.claw.setOpen();
                    robot.depoHslide.setInit();
                    follower.followPath(curve6);
                    follower.setMaxPower(0.9f);
                    vslideGoBottom = true;
                    robot.depoTilt.setWallAuto();
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    robot.claw.setClose();
                    waitFor(200);
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid, 1f);
                    robot.depoTilt.setSpec();
                    robot.depoHslide.setOut();
                    follower.setMaxPower(0.9f);
                    follower.followPath(curve7);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    robot.claw.setOpen();
                    robot.depoHslide.setInit();
                    follower.followPath(curve8);
                    follower.setMaxPower(0.9f);
                    vslideGoBottom = true;
                    robot.depoTilt.setWallAuto();
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    robot.claw.setClose();
                    waitFor(200);
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid, 1f);
                    robot.depoTilt.setSpec();
                    robot.depoHslide.setOut();
                    follower.setMaxPower(0.9f);
                    follower.followPath(curve9);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    robot.claw.setOpen();
                    robot.depoHslide.setInit();
                    follower.followPath(curve10);
                    follower.setMaxPower(0.9f);
                    vslideGoBottom = true;
                    robot.depoTilt.setWallAuto();
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    robot.claw.setClose();
                    waitFor(200);
                    robot.vSlides.moveEncoderTo(robot.vSlides.mid, 1f);
                    robot.depoTilt.setSpec();
                    robot.depoHslide.setOut();
                    follower.setMaxPower(0.9f);
                    follower.followPath(curveLast);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    robot.claw.setOpen();
                    robot.depoHslide.setInit();
                    follower.followPath(curve10);
                    follower.setMaxPower(1f);
                    vslideGoBottom = true;
                    robot.depoTilt.setTransfer();
                    setPathState(100);
                }
                break;
            case 100: //TODO: test
                telems.addLine("CASE 100 - IN TEST CASE!!");
                break;
        }
    }


    // path setter
    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        autoPath();
    }

    @Override
    public void loop() {
        follower.update();
        robot.vSlides.update();
        autoPath();
        telemetry.addLine("TValue: " + follower.getCurrentTValue());
        telemetry.addData("Slide encoder currentr: ", robot.vSlides.vSlidesR.getCurrentPosition());
        telemetry.addData("Slide encoder currentl: ", robot.vSlides.vSlidesL.getCurrentPosition());
        telemetry.addLine("Case: " + pathState);
        telemetry.addLine("PathTimer: " + pathTimer);
        telemetry.addLine("Position: " + follower.getPose());
        telemetry.addLine("heading: " + follower.getTotalHeading());

        //functions
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
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        //follower init
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        //Pose init
        buildPoses();
        buildPaths();

        //robot init
        robot.vSlides.vSlidesR.resetPosition();
        robot.vSlides.vSlidesL.resetPosition();
        vlimitswitch = hardwareMap.get(DigitalChannel.class, "vlimitswitch");
        robot.intakeTilt.setTransfer();
        robot.depoTilt.setSpec();
        waitFor(2000);
        robot.claw.setMoreClose();
        /*robot.vSlides.setUseSquID(true, vSlides.mid, 1f);
        telemetry.addData("target",robot.vSlides.getTarget());*/
        telemetry.addData("Slide encoder currentr: ", robot.vSlides.vSlidesR.getCurrentPosition());
        telemetry.addData("Slide encoder currentl: ", robot.vSlides.vSlidesL.getCurrentPosition());
    }

    //loop de loop but initialized
   /* @Override
    public void init_loop() {

    }*/

    @Override
    public void start() {
        // starts auto paths
        opmodeTimer.resetTimer();
        setPathState(1);
        // safety net if auto doesn't start for some reason
        autoPath();
    }

    public static void waitFor(int milliseconds) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < milliseconds) {
        }
    }
}
