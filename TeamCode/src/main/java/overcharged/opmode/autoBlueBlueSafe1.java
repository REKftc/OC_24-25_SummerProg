package overcharged.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import overcharged.components.RobotMecanum;
import overcharged.drive.SampleMecanumDrive;
import overcharged.pedroPathing.follower.Follower;
import overcharged.pedroPathing.localization.Pose;
import overcharged.pedroPathing.pathGeneration.BezierCurve;
import overcharged.pedroPathing.pathGeneration.BezierLine;
import overcharged.pedroPathing.pathGeneration.Path;
import overcharged.pedroPathing.pathGeneration.Point;
import overcharged.pedroPathing.util.Timer;

// Main Class
@Disabled
@Autonomous(name = "blue blue 1+1", group = "Autonomous")
public class autoBlueBlueSafe1 extends OpMode{

    // Init
    private RobotMecanum robot;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SampleMecanumDrive drive;
    MultipleTelemetry telems;
    private Timer pathTimer, opmodeTimer, scanTimer, distanceSensorUpdateTimer, distanceSensorDecimationTimer;

    // Other init
    private int pathState;

    // LOCATIONS
    // GUIDE:
    // (0,0) is the corner. (144, 144) is the opposite.

    // TODO: Here are all the Sample Poses
    // Blue side Blue Element Poses
    //
    private Pose blueBlueLeftSample = new Pose();
    private Pose blueBlueMidSample = new Pose();
    private Pose blueBlueRightSample = new Pose();
    // Blue side Neutral Element Poses
    //
    private Pose blueNeutLeftSample = new Pose();
    private Pose blueNeutMidSample = new Pose();
    private Pose blueNeutRightSample = new Pose();
    // Red Side Red Element Poses
    //
    private Pose redRedLeftSample = new Pose();
    private Pose redRedMidSample = new Pose();
    private Pose redRedRightSample = new Pose();
    // Red Side Neutral Element Poses
    //
    private Pose redNeutLeftSample = new Pose();
    private Pose redNeutMidSample = new Pose();
    private Pose redNeutRightSample = new Pose();

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
    private Pose moveOutPoint, beforeFirst, aBitForward, goToBBLlineUp, goToBBL, startToObs;
    private Pose startPose = new Pose(9.5+48, 144-7, 0);

    private Path initialObs, firstMoveToPos, findSpotOne, firstCycleToObs;

    private Follower follower;

    public void firstSpecimen(){
        startToObs = new Pose(24, 144-7, 0);
    }
    public void startingGoalPose(){
        moveOutPoint = new Pose(22, 120, 0);
        beforeFirst = new Pose(40, 72+6, 0);
    }
    public void startingGoalPoseExt(){
        aBitForward = new Pose(24, 88, 0);
    }
    public void firstCycle(){
        goToBBLlineUp = new Pose(24, 100, 0);
        goToBBL = new Pose(24, 124, 0);
    }

    public void buildPaths() {
        initialObs = new Path(new BezierLine(new Point(startPose), new Point(startToObs)));

        firstMoveToPos = new Path(new BezierCurve(new Point(startToObs), new Point(moveOutPoint), new Point(beforeFirst)));
        firstMoveToPos.setConstantHeadingInterpolation(0);

        findSpotOne = new Path(new BezierLine(new Point(beforeFirst), new Point(aBitForward)));
        findSpotOne.setConstantHeadingInterpolation(0);

        firstCycleToObs = new Path(new BezierCurve(new Point(beforeFirst), new Point(goToBBLlineUp), new Point(goToBBL)));
        firstCycleToObs.setConstantHeadingInterpolation(0);


    }


    // TODO: HERE IS WHERE THE MAIN PATH IS
    // Main pathing
    public void autoPath() {
        switch (pathState) {
            // Auto Body
            //
            case 10: // starts following the spike mark detected
                follower.followPath(firstMoveToPos);
                setPathState(11);
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(findSpotOne);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(firstCycleToObs);
                    setPathState(100);
                }
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
        startingGoalPose();
        startingGoalPoseExt();
        firstCycle();
        firstSpecimen();
        buildPaths();

        //follower init
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
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
}
