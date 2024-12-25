package overcharged.opmode;
import static overcharged.config.RobotConstants.TAG_SL;
import static overcharged.config.RobotConstants.TAG_T;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.RobotLog;
import overcharged.components.Button;
import overcharged.components.RobotMecanum;
import overcharged.pedroPathing.follower.Follower;
import overcharged.pedroPathing.pathGeneration.Vector;

// BUTTON MAPPING
/// CONTROLLER 1
// JOYSTICKS - DRIVING
// rBumper - hSlides
// lBumper - vSlides Slight down
// A - latch
/// CONTROLLER 2
// lBumper - hSlides in/Transfer
// rBumper - Transfer
// rTrigger - intake
// lTrigger - outtake
// left joystick - hSlides
// dpad right - wall vslides
// dpad left - mid vslides
// dpad up - high vslides
// dpad down - reset vslides
// A - depo claw

@Config
@TeleOp(name="teleop", group="Teleop")

public class teleop extends OpMode {
    RobotMecanum robot;
    double slowPower = 0.9f;
    int level;
    long depoTiltDelay;
    long startTime;
    long intakeTiltDelay;
    boolean intakeDelay = false;
    boolean hSlideGoBottom = false;
    boolean depohSlideOut = false;
    boolean intakeOn = false;
    boolean clawOpen = true;
    boolean latch = false;
    boolean hSlideisOut = false;
    boolean intakeTransfer = false;
    private DigitalChannel hlimitswitch;
    private DigitalChannel vlimitswitch;
    boolean firstLoop = true;
    boolean slideGoBottom = false;
    boolean hangOut = false;
    SlideLength slideLength = SlideLength.IN;
    SlideHeight slideHeight = SlideHeight.DOWN;
    IntakeMode intakeMode = IntakeMode.OFF;
    ButtonState buttonState = ButtonState.NO2;
    public enum IntakeMode {
        IN,
        OUT,
        OFF;
    }
    public enum SlideLength {
        IN,
        MID,
        LONG;
    }
    public enum SlideHeight {
        DOWN,
        WALL,
        MID,
        HIGH1,
        HIGH2;
    }
    public enum ButtonState{
        PRESSED,
        PRESSING,
        NO,
        NO2;
    }
    @Override
    public void init() {
        try {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            robot = new RobotMecanum(this, false, false);
            hlimitswitch = hardwareMap.get(DigitalChannel.class, "hlimitswitch");
            vlimitswitch = hardwareMap.get(DigitalChannel.class, "vlimitswitch");
            startTime = System.currentTimeMillis();
            robot.setBulkReadManual();

            //robot.vSlides.vSlidesB.setTargetPositionPIDFCoefficients(21,0,0,0);
        } catch (Exception e) {
            RobotLog.ee(TAG_T, "Teleop init failed: " + e.getMessage());
            telemetry.addData("Init Failed", e.getMessage());
            telemetry.update();
        }
    }
    @Override
    public void loop() {
        if(firstLoop) {
            robot.intakeTilt.setInit();
            firstLoop = false;
            robot.latch.setInit();
            robot.hslides.in();
            robot.depoHslide.setInit();
            robot.hslides.setPower(0);
            robot.vSlides.moveEncoderTo(robot.vSlides.START, 1);
        }
        robot.clearBulkCache();
        long timestamp = System.currentTimeMillis();
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = ((y + x + rx) / denominator) * slowPower;
        double backLeftPower = ((y - x + rx) / denominator) * slowPower;
        double frontRightPower = ((y - x - rx) / denominator) * slowPower;
        double backRightPower = ((y + x - rx) / denominator) * slowPower;
        robot.driveLeftFront.setPower(frontLeftPower);
        robot.driveLeftBack.setPower(backLeftPower);
        robot.driveRightFront.setPower(frontRightPower);
        robot.driveRightBack.setPower(backRightPower);

        if(slideLength==SlideLength.LONG||slideLength==SlideLength.MID){
            slowPower = 0.9f;
        }
        // Bring hSlides out
       /* if (gamepad1.right_bumper && Button.BTN_HORIZONTAL.canPress(timestamp)) {
            if (slideLength==SlideLength.IN||slideLength == SlideLength.LONG) {
                hSlideisOut = true;
                robot.latch.setOut();
                slideLength = SlideLength.MID;
                robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.hslides.moveEncoderTo(300, 0.6f);
            }
            else if (slideLength==SlideLength.MID){
                hSlideisOut = true;
                robot.latch.setOut();
                latch=false;
                slideLength = SlideLength.LONG;
                robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.hslides.moveEncoderTo(500, 0.6f);
            }

        }*/
        if (gamepad1.right_bumper && Button.TRANSFER.canPress(timestamp)) {//bumper && Button.INTAKEOUT.canPress(timestamp)){
            if (!intakeTransfer) {
                robot.intakeTilt.setTransfer();
                intakeTransfer = true;
            } else {
                robot.intakeTilt.setOut();
                intakeTransfer = false;
            }
        }

        //latch
        if (gamepad2.y && Button.BTN_LATCH.canPress(timestamp)) {
            if(!latch) {
                robot.latch.setInit();
                latch = true;
            }
            if(latch){
                robot.latch.setOut();
                latch = false;
            }
        }

        if (gamepad2.left_bumper && Button.BTN_HORIZONTAL.canPress(timestamp)) {
                hSlideGoBottom = true;
                slideLength = SlideLength.IN;
                robot.intakeTilt.setTransfer();
                intakeTransfer = true;

        }

        if (gamepad2.a && Button.CLAW.canPress(timestamp)) {
            if(!clawOpen) {
                robot.claw.setOpen();
                clawOpen = true;
            }
            else if(clawOpen){
                robot.claw.setClose();
                clawOpen = false;
            }
        }

        if (gamepad2.b && Button.BTN_OUT.canPress(timestamp)) {
            if(!depohSlideOut) {
                robot.depoHslide.setOut();
                depohSlideOut = true;
            }
            else if(depohSlideOut){
                robot.depoHslide.setInit();
                depohSlideOut = false;
            }
        }

        // Bring hSlides in
        if (!hlimitswitch.getState() && (hSlideGoBottom)) {// && robot.vSlides.getCurrentPosition() > robot.vSlides.start){//!robot.vSlides.slideReachedBottom()){
            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.hslides.hslides.setPower(-1);
            RobotLog.ii(TAG_SL, "Going down");
        } else if (hlimitswitch.getState() && (hSlideGoBottom)) {
            robot.latch.setInit();
            latch = true;
            robot.hslides.setPower(0f);
            robot.hslides.forceStop();
            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hSlideGoBottom = false;
            hSlideisOut = false;
            slideLength=SlideLength.IN;
            RobotLog.ii(TAG_SL, "Force stopped");
        }
        // Change intake tilt
        if (gamepad2.right_bumper && Button.TRANSFER.canPress(timestamp)) {//bumper && Button.INTAKEOUT.canPress(timestamp)){
            //transfer pos
            if (!intakeTransfer) {
                robot.intakeTilt.setTransfer();
                robot.depoHslide.setInit();
                robot.clawBigTilt.setInit();
                robot.clawSmallTilt.setInit();
                intakeDelay = true;
                intakeTiltDelay = System.currentTimeMillis();
                intakeTransfer = true;
            } else {
                robot.claw.setOpen();
                robot.depoHslide.setInit();
                robot.clawBigTilt.setInit();
                robot.clawSmallTilt.setInit();
                clawOpen = true;
                robot.intakeTilt.setOut();
                intakeTransfer = false;
            }
        }
        // close claw after transfer goes in
        if(intakeDelay && System.currentTimeMillis() - intakeTiltDelay > 500){
            robot.claw.setClose();
            clawOpen=false;
            intakeDelay = false;
        }
        // Intake On and Off (in)
        if (gamepad2.right_trigger > 0.9 && Button.INTAKE.canPress(timestamp)) {//gamepad1.right_bumper && Button.INTAKE.canPress(timestamp)){
            if (intakeMode == IntakeMode.OFF||intakeMode == IntakeMode.OUT) {
                robot.intake.in();
                intakeMode = IntakeMode.IN;
            } else{
                robot.intake.off();
                intakeMode = IntakeMode.OFF;
            }
        }

        if (gamepad2.y && Button.DEPOTILT.canPress(timestamp)) { // Transfer
            robot.depoHslide.setInit();
            robot.clawBigTilt.setTransfer();
            robot.claw.setOpen();
            robot.clawSmallTilt.setTransfer();
        }

        if(gamepad2.left_trigger > 0.9 && Button.INTAKEOUT.canPress(timestamp)){//bumper && Button.INTAKEOUT.canPress(timestamp)){
            if(intakeMode == IntakeMode.OFF|| intakeMode == IntakeMode.IN) {
                robot.intake.out();
                intakeMode = IntakeMode.OUT;
            }
            else {
                robot.intake.off();
                intakeMode = IntakeMode.OFF;
            }
        }
        //joy stick controls

        if(gamepad2.left_stick_y > 0.1 && !hlimitswitch.getState()) {
            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.hslides.moveEncoderTo(robot.hslides.hslides.getCurrentPosition()-(int)gamepad2.left_stick_y / 2, 0.3f);
            robot.hslides.hslides.setPower(-gamepad2.left_stick_y / 2);
        }
        if(gamepad2.left_stick_y < -0.1 && robot.hslides.hslides.getCurrentPosition() < 500){
            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.hslides.moveEncoderTo(robot.hslides.hslides.getCurrentPosition()-(int)gamepad2.left_stick_y / 2, 0.3f);
            robot.hslides.hslides.setPower(-gamepad2.left_stick_y);
        }
        if(gamepad2.left_stick_y < -0.1 && robot.hslides.hslides.getCurrentPosition() < 500){
            hSlideGoBottom = true;
        }
        if(gamepad2.left_stick_y < -0.1 && hlimitswitch.getState()){
            robot.latch.setInit();
            robot.hslides.setPower(0f);
            robot.hslides.forceStop();
            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hSlideGoBottom = false;
            hSlideisOut = false;
            slideLength=SlideLength.IN;

        }

        //Pick up from specimen
        if (gamepad2.dpad_right && Button.WALL.canPress(timestamp)) {
            robot.claw.setOpen();
            clawOpen = true;
            robot.vSlides.moveEncoderTo(robot.vSlides.wall, 1);
            slideHeight = SlideHeight.WALL;
           // depoTiltDelay = System.currentTimeMillis(); // Delay depo so it doesnt hit anything
        }
        if(slideHeight == SlideHeight.WALL && System.currentTimeMillis()-depoTiltDelay==500){
            robot.depoHslide.setInit();
            robot.clawBigTilt.setPosition(robot.clawBigTilt.WALL);
        }


        // go to bar

        if (gamepad2.dpad_left && Button.BTN_MID.canPress(timestamp)) {
            robot.vSlides.moveEncoderTo(robot.vSlides.mid, 1);
            slideHeight = SlideHeight.MID;
            //depoTiltDelay = System.currentTimeMillis();
        }
        if(slideHeight == SlideHeight.MID && System.currentTimeMillis()-depoTiltDelay==500){
            robot.depoHslide.setInit();
            robot.clawBigTilt.setPosition(robot.clawBigTilt.FLAT);
            robot.clawSmallTilt.setFlat();
        }
        // Bucket

        if (gamepad2.dpad_up && Button.HIGH1.canPress(timestamp)) {
            if(!(slideHeight == SlideHeight.HIGH1)) {
                slideHeight = SlideHeight.HIGH1;}
            robot.vSlides.moveEncoderTo(robot.vSlides.high1, 0.8f);
            //robot.vSlides.setPower(1f);
            depoTiltDelay = System.currentTimeMillis();
        }


        if(!(slideHeight == SlideHeight.HIGH2)) {
                slideHeight = SlideHeight.HIGH2;
                robot.vSlides.moveEncoderTo(robot.vSlides.high2, 1);
                robot.depoHslide.setInit();
                robot.clawBigTilt.setPosition(robot.clawBigTilt.BUCKET);
            }



        if(slideHeight == SlideHeight.HIGH1 && System.currentTimeMillis()-depoTiltDelay==500){
            robot.depoHslide.setInit();
            robot.clawBigTilt.setPosition(robot.clawBigTilt.BUCKET);
            robot.clawSmallTilt.setFlat();
        }
        //Slides down
        if (gamepad2.dpad_down && Button.WALL.canPress(timestamp)) {
            slideGoBottom = true;
            depoTiltDelay = System.currentTimeMillis();
            robot.depoHslide.setInit();
            robot.clawBigTilt.setInit();
        }
        if(slideGoBottom && System.currentTimeMillis() - depoTiltDelay > 500){
            slideBottom();
        }

        //slight down (pressing)
        if((gamepad1.left_bumper && slideHeight != SlideHeight.DOWN)){
            if(buttonState == ButtonState.NO2) {
                buttonState = ButtonState.PRESSED;
            }
            else {
                buttonState = ButtonState.PRESSING;
            }
        } else if(slideHeight != SlideHeight.DOWN){
            if(buttonState == ButtonState.PRESSING)
                buttonState = ButtonState.NO;
            else
                buttonState = ButtonState.NO2;
        }
        if(slideHeight != SlideHeight.DOWN) {
            if (slideHeight == SlideHeight.WALL)
                level = robot.vSlides.wall;
            else if (slideHeight == SlideHeight.WALL)
                level = robot.vSlides.wall;
            else if (slideHeight == SlideHeight.MID)
                level = robot.vSlides.mid;
            else if (slideHeight == SlideHeight.HIGH2)
                level = robot.vSlides.high2;
            else if (slideHeight == SlideHeight.HIGH1)
                level = robot.vSlides.high1;
        }
        if(buttonState == ButtonState.PRESSED){
            //robot.vSlides.moveEncoderTo(level-195, 0.7f);
        } else if(buttonState == ButtonState.NO){
           // robot.vSlides.moveEncoderTo(level,0.9f);
        }



        telemetry.addData("h limit switch: ",   hlimitswitch.getState());
        telemetry.addData("v limit switch: ",   vlimitswitch.getState());
        telemetry.addData("vslideRPower:", robot.vSlides.vSlidesL.getPower());
        telemetry.addData("vslideLPower:", robot.vSlides.vSlidesR.getPower());
        telemetry.addData("vslideLencoder: ", robot.vSlides.vSlidesL.getCurrentPosition());
        telemetry.addData("vslideRencoder: ", robot.vSlides.vSlidesR.getCurrentPosition());
        telemetry.addData("power:", robot.hslides.hslides.getPower());
        telemetry.addData("go back:", hSlideGoBottom);
        telemetry.addData("pos: ", robot.hslides.hslides.getCurrentPosition());
        telemetry.addData("driveLF", robot.driveLeftFront.getCurrentPosition());
        telemetry.addData("driveLB", robot.driveLeftBack.getCurrentPosition());
        telemetry.addData("driveRF", robot.driveRightFront.getCurrentPosition());
        telemetry.addData("intake", robot.intake.intake.getCurrentPosition());
        telemetry.addData("driveRB", robot.driveRightBack.getCurrentPosition());
        telemetry.addData("hslidePower", robot.hslides.getPower());

        telemetry.update();
    }
    public void slideBottom() {
        if (vlimitswitch.getState()) {
            robot.vSlides.vSlidesL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.vSlides.vSlidesR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.vSlides.setPower(-1f);
            RobotLog.ii(TAG_SL, "Going down");
        } else {
            robot.vSlides.setPower(0f);
            robot.vSlides.vSlidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.vSlides.vSlidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideGoBottom = false;
            robot.intake.in();
            robot.claw.setOpen();
            clawOpen = true;
            RobotLog.ii(TAG_SL, "Force stopped");
        }
    }
}
