package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
import overcharged.pedroPathing.pathGeneration.Point;
import overcharged.pedroPathing.util.Timer;

// Main Class
//@Disabled
@Autonomous(name = "GAMBLINGGGG", group = "Autonomous")
public class autoSubGambling extends OpMode{

    //vars
    boolean yes = false;
    boolean shake = false;
    boolean vslideGoBottom = false;
    boolean hSlideGoBottom = false;

    boolean oneTime = false;

    int turnCoeff = 1;



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

    // OTHER POSES
    private Pose staySigma;
    private Pose startPose = new Pose(84, 49.5, Math.toRadians(90));

    private Path woahLigma, slightMove;

    private Follower follower;

    public void Paths(){
        staySigma = new Pose(84, 49.5, Math.toRadians(90));
    }

    public void buildPaths() {
        slightMove = new Path(new BezierLine(new Point(staySigma.getX()+(turnCoeff-1)*2, staySigma.getY()), new Point(staySigma.getX()+(turnCoeff)*2, staySigma.getY())));
    }


    // TODO: HERE IS WHERE THE MAIN PATH IS
    // Main pathing
    public void autoPath() {
        switch (pathState) {
            // Auto Body
            //
            case 10: // starts following the spike mark detected
                robot.latch.setOut();
                robot.hslides.moveEncoderTo(hslides.SMALL,1f);
                waitFor(200);
                setPathState(11);
                break;
            case 11: ;
                follower.followPath(slightMove);
                slightMove.setConstantHeadingInterpolation(Math.toRadians(90));
                waitFor(300);
                robot.trapdoor.setInit();
                robot.intake.in();
                robot.intakeTilt.setOut();
                pathTimer.resetTimer();
                setPathState(12);
                break;
            case 12:
                robot.hslides.moveEncoderTo((int)robot.hslides.hslides.getCurrentPosition() + 120, 1f);
                if(robot.sensorF.getColor() == colorSensor.Color.RED || robot.sensorF.getColor() == colorSensor.Color.YELLOW){
                    robot.intakeTilt.setTransfer();
                    setPathState(13);
                }
                else if (robot.sensorF.getColor() == colorSensor.Color.BLUE){
                    robot.intakeTilt.setFlat();
                    robot.trapdoor.setOut();
                    turnCoeff += 1;
                    setPathState(11);
                }
                else if (robot.hslides.hslides.getCurrentPosition() > 600) {
                    robot.intake.out();
                    turnCoeff += 1;
                    setPathState(121);
                }
                else if (pathTimer.getElapsedTime()>3000){
                    robot.intake.out();
                    turnCoeff += 1;
                    setPathState(121);
                }
                break;
            case 121:
                hSlideGoBottom = true;
                waitFor(200);
                robot.intake.in();
                setPathState(11);
                break;
            case 13:
                waitFor(200);
                robot.intake.out();
                waitFor(200);
                hSlideGoBottom = true;
                waitFor(300);
                robot.intake.off();
                setPathState(100);
                break;
            case 100: // EMPTY TEST CASE
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

        if (!hlimitswitch.getState() && hSlideGoBottom) {
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
        robot = new RobotMecanum(this, true, true);
        drive = new SampleMecanumDrive(hardwareMap);
        pathTimer = new Timer();

        //Pose init
        Paths();
        buildPaths();

        checkC();

        //follower init
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        hlimitswitch = hardwareMap.get(DigitalChannel.class, "hlimitswitch");
        vlimitswitch = hardwareMap.get(DigitalChannel.class, "vlimitswitch");
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

    public void checkC(){
        if(robot.sensorF.getColor() == colorSensor.Color.RED){
            yes = true;
        }
        else if(robot.sensorF.getColor() == colorSensor.Color.YELLOW){
            yes = true;
        }
        else if(robot.sensorF.getColor() == colorSensor.Color.BLUE){
            yes = false;
        }
        else{

        }
    }

    public void shaker(){
        shake = !shake;
        while (shake){
            follower.followPath(new Path( new BezierLine(new Point(follower.getPose().getX(),follower.getPose().getY()),new Point(follower.getPose().getX()-1,follower.getPose().getY()))));
            waitFor(500);
            follower.followPath(new Path( new BezierLine(new Point(follower.getPose().getX(),follower.getPose().getY()),new Point(follower.getPose().getX()+1,follower.getPose().getY()))));
            waitFor(500);
        }
    }


}
