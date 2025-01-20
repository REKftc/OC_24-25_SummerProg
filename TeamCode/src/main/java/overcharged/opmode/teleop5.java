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
import overcharged.components.colorSensor;
import overcharged.components.hslides;
import overcharged.components.vSlides;
import overcharged.pedroPathing.follower.Follower;
import overcharged.pedroPathing.pathGeneration.Vector;


@Config
@TeleOp(name="new tele", group="Teleop")
public class teleop5 extends OpMode{

    RobotMecanum robot;
    private DigitalChannel hlimitswitch;
    private DigitalChannel vlimitswitch;

    long depoDelay;
    long clawDelay;

    double slowPower = 1;
    float turnConstant = 1f;

    int wallStep = 0;
    int resetStep = 0;
    int transferStep = 0;

    boolean intakeTransfer = true;
    boolean clawOpen = true;
    boolean dDelay = false;
    boolean cDelay = false;
    boolean bucketSeq = false;

    boolean vslideGoBottom = false;
    boolean hSlideGoBottom = false;

    IntakeMode intakeMode = IntakeMode.OFF;
    SlideHeight slideHeight = SlideHeight.DOWN;

    public enum IntakeMode {
        IN,
        OUT,
        OFF;
    }

    public enum SlideHeight {
        DOWN,
        WALL,
        MID,
        LOWER,
        HIGH1,
        HIGH2;
    }

    public void init() {
        try {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            robot = new RobotMecanum(this, false, false);
            hlimitswitch = hardwareMap.get(DigitalChannel.class, "hlimitswitch");
            vlimitswitch = hardwareMap.get(DigitalChannel.class, "vlimitswitch");
            robot.setBulkReadManual();
        } catch (Exception e) {
            RobotLog.ee(TAG_T, "Teleop init failed: " + e.getMessage());
            telemetry.addData("Init Failed", e.getMessage());
            telemetry.update();
        }
    }

    public void loop(){
        robot.clearBulkCache();
        long timestamp = System.currentTimeMillis();

        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x*turnConstant;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = ((y + x + rx) / denominator) * slowPower;
        double backLeftPower = ((y - x + rx) / denominator) * slowPower;
        double frontRightPower = ((y - x - rx) / denominator) * slowPower;
        double backRightPower = ((y + x - rx) / denominator) * slowPower;

        robot.driveLeftFront.setPower(frontLeftPower);
        robot.driveLeftBack.setPower(backLeftPower);
        robot.driveRightFront.setPower(frontRightPower);
        robot.driveRightBack.setPower(backRightPower);


        if(gamepad1.right_bumper && Button.SLIDE_RESET.canPress(timestamp)){
            robot.latch.setOut();
            turnConstant = 0.70f;
            robot.hslides.moveEncoderTo(robot.hslides.OUT,1f);
        }

        if (gamepad1.left_bumper && Button.TRANSFER.canPress(timestamp)) {
            if (!intakeTransfer) {
                robot.intakeTilt.setTransfer();
                intakeTransfer = true;
            } else {
                robot.intakeTilt.setInOut();
                robot.intake.in();
                intakeTransfer = false;
            }
        }

        if (gamepad1.right_trigger > 0.9 && Button.INTAKE.canPress(timestamp)) {
            if (intakeMode == IntakeMode.OFF ||intakeMode == IntakeMode.OUT) {
                robot.intake.in();
                intakeMode = IntakeMode.IN;
            } else{
                robot.intake.off();
                intakeMode = IntakeMode.OFF;
            }
        }
        if(gamepad1.left_trigger > 0.9 && Button.INTAKEOUT.canPress(timestamp)){
            if(intakeMode == IntakeMode.OFF || intakeMode == IntakeMode.IN) {
                robot.intake.out();
                intakeMode = IntakeMode.OUT;
            }
            else {
                robot.intake.off();
                intakeMode = IntakeMode.OFF;
            }
        }

        if(gamepad1.y && Button.TRANSFER.canPress(timestamp)){
            transferNow();
        }

        if (gamepad2.a && Button.CLAW.canPress(timestamp)) { // claw
            if(!clawOpen) {
                robot.claw.setOpen();
                clawOpen = true;
            }
            else if(clawOpen){
                robot.claw.setClose();
                clawOpen = false;
            }
        }

        if(gamepad2.dpad_up && Button.HIGH1.canPress(timestamp)){ //vSlides Up to Bucket
            robot.claw.setClose();
            clawOpen = false;

            slideHeight = SlideHeight.HIGH1;
            robot.vSlides.moveEncoderTo(robot.vSlides.high1, 1f);
        }

        if (gamepad2.left_bumper && Button.BTN_LEVEL2.canPress(timestamp)){ // Lower Bucket
            robot.claw.setClose();
            clawOpen = false;

            slideHeight = SlideHeight.LOWER;
            robot.vSlides.moveEncoderTo(robot.vSlides.lower, 1f);
        }

        if(gamepad2.dpad_left && Button.BTN_MID.canPress(timestamp)){ // High Specimen
            robot.claw.setClose();
            clawOpen = false;

            slideHeight = SlideHeight.MID;
            robot.vSlides.moveEncoderTo(robot.vSlides.mid+50, 1f);
        }

        if(gamepad2.dpad_right && Button.WALL.canPress(timestamp)) {
            robot.claw.setClose();
            clawOpen = false;

            slideHeight = SlideHeight.WALL;

            wallStep = 0;
            depoDelay = System.currentTimeMillis();
            wallStep++;
        }

        if(gamepad2.dpad_down && Button.SLIDE_RESET.canPress(timestamp)) { // Slide reset
            robot.depoHslide.setInit();
            if(slideHeight == SlideHeight.DOWN || slideHeight == SlideHeight.WALL || slideHeight == SlideHeight.LOWER) {
                robot.depoWrist.setIn();
                robot.intakeTilt.setFlat();
                robot.clawBigTilt.setFlat();
                robot.clawSmallTilt.setTranSeq();

                depoDelay = System.currentTimeMillis();
                resetStep++;
            }
            else{
                robot.depoWrist.setIn();
                robot.clawSmallTilt.setTransfer();
                robot.clawBigTilt.setTransfer();
                robot.intakeTilt.setTransfer();

                slideHeight = SlideHeight.DOWN;
                depoDelay = System.currentTimeMillis();
                dDelay = true;
            }
        }

        if(vslideGoBottom){ //Reset vSlide check
            slideBottom();
        }

        if(intakeTransfer && cDelay && System.currentTimeMillis()-clawDelay>100){ // Transfer System
            cDelay = false;
            robot.depoWrist.setIn();
            robot.clawBigTilt.setTransfer();
            robot.clawSmallTilt.setTransfer();
            robot.intakeTilt.setTransfer();

            transferStep = 0;
            transferStep++;
            clawDelay = System.currentTimeMillis();
        }
        if (transferStep ==1 & System.currentTimeMillis()-clawDelay>150){
            robot.claw.setClose();
            clawOpen = false;

            transferStep = 0;
            clawDelay = 0;
        }
        // Bucket(High & Low) sequence
        if (slideHeight == SlideHeight.HIGH1 && System.currentTimeMillis()-depoDelay>200 &dDelay || slideHeight == SlideHeight.LOWER && System.currentTimeMillis()-depoDelay>200 &dDelay) { // Depo to Bucket
            robot.clawSmallTilt.setOut();
            robot.depoHslide.setInit();
            bucketSeq = true;

            depoDelay = System.currentTimeMillis();
            dDelay = false;
        }
        if (bucketSeq && slideHeight == SlideHeight.HIGH1 && System.currentTimeMillis()-depoDelay>100 || bucketSeq && slideHeight == SlideHeight.LOWER && System.currentTimeMillis()-depoDelay>70){
            bucketSeq = false;
            robot.clawBigTilt.setBucket();
            robot.intakeTilt.setTransfer();
            robot.depoWrist.setOut();

            depoDelay = 0;
        }
        if (slideHeight == SlideHeight.MID && System.currentTimeMillis()-depoDelay>500 && dDelay) { // Depo to Specimen
            robot.claw.setSpec();
            robot.clawBigTilt.setOut();
            robot.depoHslide.setOut();
            robot.clawSmallTilt.setFlat();

            depoDelay = 0;
            dDelay = false;
        }

        // Wall pickup Sequence
        if(wallStep==1 && System.currentTimeMillis() - depoDelay > 50){
            robot.claw.setClose();
            clawOpen = false;

            robot.intakeTilt.setFlat();
            robot.clawBigTilt.setFlat();
            robot.clawSmallTilt.setTranSeq();
            robot.clawBigTilt.setWall();
            robot.depoHslide.setInit();

            depoDelay = System.currentTimeMillis();
            wallStep++;
        }
        if(wallStep==2 && System.currentTimeMillis() - depoDelay > 180){
            robot.claw.setClose();
            clawOpen = false;

            robot.clawSmallTilt.setWall();
            robot.clawBigTilt.setWall();

            depoDelay = System.currentTimeMillis();
            wallStep++;
        }
        if(wallStep==3 && System.currentTimeMillis() - depoDelay > 280){
            robot.claw.setOpen();
            clawOpen = true;

            wallStep=0;
            depoDelay = 0;
        }

        //slide reset seq
        if(slideHeight == SlideHeight.DOWN && System.currentTimeMillis()-depoDelay>150 && dDelay){
            vslideGoBottom = true;
            depoDelay =0;
            dDelay =false;
        }
        if(resetStep==1 && System.currentTimeMillis() - depoDelay > 300){
            robot.clawSmallTilt.setTransfer();
            robot.clawBigTilt.setTransfer();

            depoDelay = System.currentTimeMillis();
            resetStep++;
        }
        if(resetStep==2 && System.currentTimeMillis() - depoDelay > 300){
            robot.intakeTilt.setTransfer();
            slideHeight = SlideHeight.DOWN;
            robot.claw.setOpen();
            clawOpen = true;

            resetStep=0;
            depoDelay = 0;
        }

        // Logic for bringing hslides back in
        if (!hlimitswitch.getState() && hSlideGoBottom) {
            turnConstant = 1f;
            robot.latch.setInit();
            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.hslides.hslides.setPower(-1f);
        } else if (hlimitswitch.getState() && hSlideGoBottom) {
            robot.intake.off();
            intakeMode = IntakeMode.OFF;
            robot.hslides.hslides.setPower(0);
            robot.latch.setInit();
            robot.intakeTilt.setTransfer();
            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            clawDelay = System.currentTimeMillis();
            cDelay = true;
            hSlideGoBottom = false;
        }
    }

    public void slideBottom() { //Slide bottom
        if (!vlimitswitch.getState() && vslideGoBottom) {
            robot.vSlides.vSlidesL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.vSlides.vSlidesR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.vSlides.vSlidesR.setPower(-1);
            robot.vSlides.vSlidesL.setPower(-1);
        } else if (vlimitswitch.getState() && vslideGoBottom) {
            robot.vSlides.vSlidesR.setPower(0);
            robot.vSlides.vSlidesL.setPower(0);
            robot.vSlides.vSlidesL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.vSlides.vSlidesR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            vslideGoBottom = false;
        }
    }

    public void transferNow(){
        robot.clawBigTilt.setTransfer();
        robot.intakeTilt.setTransfer();
        robot.latch.setInit();
        robot.depoWrist.setIn();
        robot.intake.in();
        intakeMode = IntakeMode.IN;
        intakeTransfer = true;
        robot.claw.setOpen();
    }

}
