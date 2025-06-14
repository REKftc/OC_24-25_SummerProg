package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

public class specCycle {

    private Follower follower;
    private RobotMecanum robot;
    private DigitalChannel hlimitswitch;
    private DigitalChannel vlimitswitch;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SampleMecanumDrive drive;
    MultipleTelemetry telems;
    private Pose startPose = new Pose(0, 0, Math.toRadians(90));
    private int pathState;
    private ElapsedTime pathTimer;

    boolean vslideGoBottom = false;

    private Pose getSpec2, getSpec2Back, scoreSpec2Mid, scoreSpec2End, scoreSpec3Mid, scoreSpec3End, scoreSpec4Mid, scoreSpec4End, scoreSpec5Mid, scoreSpec5End, park;

    private Path curve5, curve6, curve7, curve8, curve9, curve10, curveLast, parkEnd;

    public void buildPoses() {
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

    public void buildPaths() {

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



    public void runOpMode(){
        Move();
        setPathState(10);
    }

    public void setPathState(int state){
        pathState = state;
        pathTimer.reset();
        Move();
    }

    public static void waitFor(int milliseconds) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < milliseconds) {
        }
    }

    public void slideReset() { //vSlide bottom
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


    public void Move() {
        switch (pathState) {
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
        }
    }
}

