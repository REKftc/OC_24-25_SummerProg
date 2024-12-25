package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import overcharged.components.RobotMecanum;
import overcharged.drive.SampleMecanumDrive;
import overcharged.linear.util.SelectLinear;

import overcharged.pedroPathing.follower.Follower;
import overcharged.pedroPathing.localization.Pose;
import overcharged.pedroPathing.pathGeneration.BezierCurve;
import overcharged.pedroPathing.pathGeneration.BezierLine;
import overcharged.pedroPathing.pathGeneration.Path;
import overcharged.pedroPathing.pathGeneration.Point;
import overcharged.pedroPathing.util.Timer;

// Main Class
@Autonomous(name = "blue blue 1+3", group = "Autonomous")
public class autoBlueBlueSafe3 extends OpMode{

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
    private Pose moveOutPoint, beforeFirst, aBitForward, goToBBLlineUp, goToBBL, startToObs, goToBBMlineUp, goToBBM, goToBBRlineUp, goToBBR;
    private Pose startPose = new Pose(6.5+48, 144-7, 0);

    private Path initialObs, firstMoveToPos, findSpotOne, firstCycleToObs, firstCycleBack, secondCycleToObs, secondCycleBack, finalCycleToObs;

    private Follower follower;

    //TODO: Starting from here are the poses for the paths
    public void firstSpecimen(){
        startToObs = new Pose(40, 144-4, 0);
    }
    public void startingGoalPose(){
        moveOutPoint = new Pose(22, 120, 0);
        beforeFirst = new Pose(40, 72+6, 0);
    }
    public void startingGoalPoseExt(){
        aBitForward = new Pose(29, 88, 0);
    }
    public void firstCycle(){
        goToBBLlineUp = new Pose(32, 100, 0);
        goToBBL = new Pose(32, 124, 0);
    }
    public void secondCycle(){
        goToBBMlineUp = new Pose(5, 84, 0);
        goToBBM = new Pose(20, 128, 0);
    }
    public void thirdCycle(){
        goToBBRlineUp = new Pose(-2, 70, 0);
        goToBBR = new Pose(10, 124, 0);
    }

    //TODO: here are where the paths are defined
    public void buildPaths() {
        initialObs = new Path(new BezierCurve(new Point(startPose), new Point(44, 144-4, 0), new Point(startToObs)));
        initialObs.setConstantHeadingInterpolation(0);
        initialObs.setPathEndTimeoutConstraint(2);

        firstMoveToPos = new Path(new BezierCurve(new Point(startToObs), new Point(moveOutPoint), new Point(beforeFirst)));
        firstMoveToPos.setConstantHeadingInterpolation(0);
        firstMoveToPos.setPathEndTimeoutConstraint(3);

        findSpotOne = new Path(new BezierLine(new Point(beforeFirst), new Point(aBitForward)));
        findSpotOne.setConstantHeadingInterpolation(0);
        findSpotOne.setPathEndTimeoutConstraint(3);

        firstCycleToObs = new Path(new BezierCurve(new Point(aBitForward), new Point(goToBBLlineUp), new Point(goToBBL)));
        firstCycleToObs.setConstantHeadingInterpolation(0);

        firstCycleBack = new Path(new BezierLine(new Point(goToBBL),new Point(aBitForward)));
        firstCycleBack.setConstantHeadingInterpolation(0);

        secondCycleToObs = new Path(new BezierCurve(new Point(aBitForward), new Point(goToBBMlineUp),new Point(goToBBM)));
        secondCycleToObs.setConstantHeadingInterpolation(0);

        secondCycleBack = new Path(new BezierLine(new Point(goToBBM), new Point(aBitForward)));
        secondCycleBack.setConstantHeadingInterpolation(0);

        finalCycleToObs = new Path(new BezierCurve(new Point(aBitForward), new Point(goToBBRlineUp), new Point(goToBBR)));
        finalCycleToObs.setConstantHeadingInterpolation(0);


    }


    // TODO: HERE IS WHERE THE MAIN PATH IS
    // Main pathing
    public void autoPath() {
        switch (pathState) {
            // Auto Body
            //
            case 10: // scores initial specimen
                follower.followPath(initialObs);
                setPathState(100);
                break;
            case 101:
                follower.followPath(firstMoveToPos);
                setPathState(11);
                break;
            case 11: //gets ready for first cycle
                if (!follower.isBusy()) {
                    follower.followPath(findSpotOne);
                    setPathState(12);
                }
                break;
            case 12: //score first cycle
                if (!follower.isBusy()) {
                    follower.followPath(firstCycleToObs);
                    setPathState(13);
                }
                break;
            case 13: //goes back and gets ready for second cycle
                if (!follower.isBusy()){
                    follower.followPath(firstCycleBack);
                    setPathState(14);
                }
                break;
            case 14: //score second cycle
                if (!follower.isBusy()){
                    follower.followPath(secondCycleToObs);
                    setPathState(15);
                }
                break;
            case 15: //goes back and gets ready for final cycle
                if (!follower.isBusy()){
                    follower.followPath(secondCycleBack);
                    setPathState(16);
                }
                break;
            case 16: //score final cycle and park
                if (!follower.isBusy()){
                    follower.followPath(finalCycleToObs);
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
        robot = new RobotMecanum(this, true, false);
        drive = new SampleMecanumDrive(hardwareMap);
        pathTimer = new Timer();

        //Pose init
        startingGoalPose();
        startingGoalPoseExt();
        firstCycle();
        firstSpecimen();
        secondCycle();
        thirdCycle();
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
