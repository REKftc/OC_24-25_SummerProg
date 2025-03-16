package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_SL;
import static overcharged.config.RobotConstants.TAG_T;

import android.transition.Slide;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.Button;
import overcharged.components.RobotMecanum;
import overcharged.components.colorSensor;
import overcharged.components.hang;
import overcharged.components.hslides;
import overcharged.components.vSlides;



@Config
@TeleOp(name="diffy red", group="!Teleop")
public class teleop7 extends OpMode{

    RobotMecanum robot;
    private DigitalChannel hlimitswitch;
    private DigitalChannel vlimitswitch;

    long depoDelay;
    long resetDelay;
    long clawDelay;
    long outakeTime;
    long intakeTiltDelay;
    long manualDelay;
    long outDelay;
    long hangDelay;
    long lastButtonPressTime = 0;
    final long debounceTime = 200;
    long clearDelay = 0;
    ElapsedTime temp;

    double slowPower = 1;
    float turnConstant = 1f;

    int wallStep = 0;
    int resetStep = 0;
    int transferStep = 0;
    int intakeStep = 0;
    int hangCheck = 0;
    int hangTick;

    boolean intakeTransfer = true;
    boolean intakeDelay = false;
    boolean intakeOn = false;
    boolean intTiltDelay = false;
    boolean clawOpen = true;
    boolean hslideOut = false;
    boolean vslideOut = false;
    boolean manualOut = false;
    boolean sense = false;
    boolean manualCheck = false;
    boolean trapOnce = true;
    boolean hangSeq = false;
    boolean runLeft = false;
    boolean ptoOn = false;
    boolean canYellow = true;

    boolean dDelay = false;
    boolean cDelay = false;
    boolean bucketSeq = false;
    boolean intakeOutDelay = false;
    boolean hang2 = false;
    boolean hang3 = false;
    boolean latch = true;
    boolean vslideGoBottom = false;
    boolean hSlideGoBottom = false;

    IntakeMode intakeMode = IntakeMode.OFF;
    SlideHeight slideHeight = SlideHeight.DOWN;
    ScoreType score = ScoreType.NONE;


    public enum IntakeMode {
        IN,
        OUT,
        OFF;
    }

    public enum ScoreType {
        BUCKET,
        SPECIMEN,
        NONE
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
        robot.setBulkReadAuto();
        temp = new ElapsedTime();
        robot.vSlides.vSlidesR.resetPosition();
        robot.vSlides.vSlidesL.resetPosition();
    }

    public void loop(){
        temp.reset();
        robot.clearBulkCache();
        long timestamp = System.currentTimeMillis();

        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x * turnConstant;
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
            robot.intakeTilt.setGoOut();
            robot.latch.setOut();
            turnConstant = 0.55f;
            robot.hslides.moveEncoderTo(robot.hslides.SMALL_OUT,1f);
            robot.intake.in();
            intakeMode = IntakeMode.IN;
            sense = true;

            manualDelay = System.currentTimeMillis();
            hslideOut = true;
            manualCheck = true;

        }

        if (manualOut) {
            float slidePower = -gamepad1.right_stick_y;
            if (Math.abs(slidePower) > 0.105) {
                robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.hslides.hslides.setPower(slidePower*0.8f);
            } else {
                robot.hslides.hslides.setPower(0);
            }
        }

        if (gamepad1.left_bumper && Button.TRANSFER.canPress(timestamp)) {
            if (!intakeTransfer) {
                clawOpen = true;
                robot.intakeTilt.setTransfer();
                intakeTransfer = true;
                if(hlimitswitch.getState() && latch) {
                    clawDelay = System.currentTimeMillis();
                    cDelay = true;
                }
            } else {
                robot.intakeTilt.setOut();
                intakeTiltDelay = System.currentTimeMillis();
                intTiltDelay = true;
                robot.intake.in();
                intakeMode = IntakeMode.IN;
                intakeTransfer = false;
            }
        }

        if (gamepad1.b && Button.BTN_SLIDE_OUT.canPress(timestamp)) {
            if (manualOut){
                manualOut = false;
                robot.intakeTilt.setOut();
                robot.hslides.moveEncoderTo(robot.hslides.hslides.getCurrentPosition()+80, 1f);
                clearDelay = System.currentTimeMillis();
            } else{
                robot.latch.setOut();
                robot.hslides.moveEncoderTo(robot.hslides.PRESET1, 1f);
                hslideOut = true;
                sense = true;
                clearDelay = System.currentTimeMillis();
            }
        }

        if(clearDelay > 400){
            robot.intakeTilt.setInOut();
            robot.hslides.moveEncoderTo(robot.hslides.hslides.getCurrentPosition()-80, 1f);
        }
        if(clearDelay > 750){
            robot.intakeTilt.setOut();
            manualOut = true;
            clearDelay = 0;
        }

        if (gamepad1.back && Button.INTAKEDOOR.canPress(timestamp)){
            canYellow = !canYellow;
        }

        if (gamepad1.right_trigger > 0.75 && Button.INTAKE.canPress(timestamp)) {
            if (intakeMode == IntakeMode.OFF ||intakeMode == IntakeMode.OUT) {
                robot.intake.in();
                intakeMode = IntakeMode.IN;
                sense = true;
            } else{
                robot.intake.off();
                intakeMode = IntakeMode.OFF;
                sense = false;
            }
        }

        if(gamepad1.left_trigger > 0.75&& Button.INTAKEOUT.canPress(timestamp)){
            sense = false;
            if(intakeMode == IntakeMode.OFF || intakeMode == IntakeMode.IN) {
                robot.intake.out();
                intakeMode = IntakeMode.OUT;
            }
            else {
                robot.intake.off();
                intakeMode = IntakeMode.OFF;
            }
        }

        if (gamepad1.x || gamepad2.right_bumper) {
            if (trapOnce && (System.currentTimeMillis() - lastButtonPressTime > debounceTime)) {
                trapOnce = false;
                robot.trapdoor.setOut(); // Open the trapdoor
                lastButtonPressTime = System.currentTimeMillis();
            }
        } else {
            if (!trapOnce && (System.currentTimeMillis() - lastButtonPressTime > debounceTime)) {
                robot.trapdoor.setInit(); // Close the trapdoor
                trapOnce = true;
                lastButtonPressTime = System.currentTimeMillis();
            }
        }

        if(gamepad1.y && Button.TRANSFER.canPress(timestamp)){
            transferNow();
        }

        if(intakeOutDelay){
            intakeOutDelay = false;
            robot.intake.in();
            intakeMode = IntakeMode.IN;
            intakeStep = 0;
            intakeStep++;
            outakeTime = System.currentTimeMillis();
        }
        if(intakeStep == 1 && System.currentTimeMillis()-outakeTime>70){
            robot.intakeTilt.setTransfer();
            robot.intake.slowOut();
            intakeMode = IntakeMode.OUT;
            intakeStep++;
            outakeTime = System.currentTimeMillis();
        }
        if(intakeStep == 2 && System.currentTimeMillis()-outakeTime>400){
            robot.intake.off();
            intakeMode = IntakeMode.OFF;
            intakeStep = 0;
            outakeTime = 0;
        }

        if(intakeTransfer && cDelay && System.currentTimeMillis()-clawDelay>90){ // Transfer System
            cDelay = false;
            robot.depoWrist.setTransfer();
            robot.clawBigTilt.setTransfer();
            robot.clawSmallTilt.setTransfer();
            transferStep = 0;
            transferStep++;
            clawDelay = System.currentTimeMillis();
        }
        if (transferStep ==1 & System.currentTimeMillis()-clawDelay>70){
            robot.intakeTilt.setTransfer();
            transferStep++;
            clawDelay = System.currentTimeMillis();
        }
        if (transferStep ==2 & System.currentTimeMillis()-clawDelay>140){
            robot.claw.setClose();
            clawOpen = false;
            transferStep = 0;
            clawDelay = 0;
        }

        if(!intakeTransfer && intTiltDelay && System.currentTimeMillis()- intakeTiltDelay>290){ //delay in submersible
            robot.intakeTilt.setOut();
            intTiltDelay = false;
        }

        if(intakeMode == IntakeMode.IN && sense){
            if (robot.sensorF.getColor() == colorSensor.Color.RED){
                intakeOn = false;
                robot.trapdoor.setInit();
                intakeMode = IntakeMode.OFF;
                robot.intake.off();
                transferNow();
                sense = false;
                hslideOut = false;
            }
            if (robot.sensorF.getColor() == colorSensor.Color.YELLOW && canYellow){
                intakeOn = false;
                robot.trapdoor.setInit();
                intakeMode = IntakeMode.OFF;
                robot.intake.off();
                transferNow();
                sense = false;
                hslideOut = false;
            } else if (robot.sensorF.getColor() == colorSensor.Color.YELLOW && !canYellow){
                intakeMode = IntakeMode.OFF;
                robot.intake.out();
                robot.trapdoor.setOut();
                robot.intakeTilt.setFlat();
                intakeOn = true;
                manualOut = false;
                intakeDelay = true;
                sense = false;
                outDelay = System.currentTimeMillis();
            }
            if (robot.sensorF.getColor() == colorSensor.Color.BLUE){
                intakeMode = IntakeMode.OFF;
                robot.intake.out();
                robot.trapdoor.setOut();
                robot.intakeTilt.setFlat();
                intakeOn = true;
                manualOut = false;
                intakeDelay = true;
                sense = false;
                outDelay = System.currentTimeMillis();
            }
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

        if(gamepad2.dpad_up && Button.HIGH1.canPress(timestamp)) { //vSlides Up to Bucket
            slowPower = 0.82f;
            robot.claw.setClose();
            clawDelay = System.currentTimeMillis();
            cDelay = true;
            turnConstant = 0.7f;
            clawOpen = false;
            slideHeight = SlideHeight.HIGH1;
        }
        if(cDelay && slideHeight == SlideHeight.HIGH1 && clawDelay > 400){
            cDelay= false;
            clawDelay = 0;
            robot.vSlides.moveEncoderTo(robot.vSlides.high1, 1f);
            dDelay = true;
            depoDelay = System.currentTimeMillis();
        }

        if (gamepad2.left_bumper && Button.BTN_LEVEL2.canPress(timestamp)){ // Lower Bucket
            slowPower = 0.82f;
            robot.claw.setClose();
            clawOpen = false;

            slideHeight = SlideHeight.LOWER;
            robot.vSlides.moveEncoderTo(robot.vSlides.lower, 1f);

            dDelay = true;
            depoDelay = System.currentTimeMillis();
        }

        if(gamepad2.dpad_left && Button.BTN_MID.canPress(timestamp)) { // High Specimen
            robot.claw.setClose();
            clawOpen = false;
            clawDelay = System.currentTimeMillis();
            cDelay = true;
            slideHeight = SlideHeight.MID;
        }
        if(cDelay && slideHeight == SlideHeight.MID && clawDelay > 400){
            cDelay= false;
            clawDelay = 0;
            vslideOut = true;
            dDelay = true;
            robot.vSlides.moveEncoderTo(robot.vSlides.mid, 1f);
            depoDelay = System.currentTimeMillis();
        }

        if(gamepad2.dpad_right && Button.WALL.canPress(timestamp)) {
            slideHeight = SlideHeight.WALL;
            robot.vSlides.moveEncoderTo(robot.vSlides.wall, 1f);
            vslideOut = true;
            wallStep = 0;
            robot.depoHslide.setInit();
            robot.claw.setClose();
            clawOpen = false;
            depoDelay = System.currentTimeMillis();
            wallStep++;
        }

        if(gamepad2.dpad_down && Button.SLIDE_RESET.canPress(timestamp)) { // Slide reset
            slowPower = 1f;
            robot.depoHslide.setInit();
            if(slideHeight == SlideHeight.DOWN || slideHeight == SlideHeight.WALL || slideHeight == SlideHeight.LOWER) {
                vslideGoBottom = true;
                robot.depoWrist.setTransfer();
                robot.claw.setClose();
                robot.intakeTilt.setInOut();
                robot.clawBigTilt.setFlat();
                robot.clawSmallTilt.setTranSeq();

                resetDelay = System.currentTimeMillis();
                resetStep++;
            } else if (slideHeight == SlideHeight.MID){
                vslideGoBottom = true;
                robot.clawSmallTilt.setTransfer();
                robot.depoWrist.setTransfer();
                robot.clawBigTilt.setTransfer();

                slideHeight = SlideHeight.DOWN;
                resetDelay = System.currentTimeMillis();
                dDelay = true;
            }
            else{
                vslideGoBottom = true;
                robot.depoWrist.setTransfer();
                robot.clawBigTilt.setTransfer();

                slideHeight = SlideHeight.DOWN;
                resetDelay = System.currentTimeMillis();
                dDelay = true;
            }
        }

        if(gamepad2.x && Button.SLIGHT_UP.canPress(timestamp)){
            if(robot.vSlides.vSlidesL.getCurrentPosition() < robot.vSlides.high1){
                robot.vSlides.moveEncoderTo((int)(robot.vSlides.vSlidesL.getCurrentPosition())+90, 0.9f);
            }
        }


        if(gamepad2.b && Button.SLIGHT_DOWN.canPress(timestamp)){
            if(robot.vSlides.vSlidesL.getCurrentPosition() > 100){
                robot.vSlides.moveEncoderTo((int)(robot.vSlides.vSlidesL.getCurrentPosition())-90, 0.3f);
            }
        }

        if(vslideGoBottom){ //Reset vSlide check
            slideBottom();
        }

        if(hslideOut && System.currentTimeMillis()-manualDelay>500 && manualCheck){
            robot.clawSmallTilt.setOut();
            manualCheck = false;
            manualDelay = 0;
            manualOut = true;
        }

        // Bucket(High & Low) sequence
        if (slideHeight == SlideHeight.HIGH1 && System.currentTimeMillis()-depoDelay>290 &dDelay || slideHeight == SlideHeight.LOWER && System.currentTimeMillis()-depoDelay>290 &dDelay) { // Depo to Bucket
            vslideOut = true;

            robot.clawSmallTilt.setOut();
            robot.depoHslide.setInit();
            score = ScoreType.BUCKET;
            bucketSeq = true;
            depoDelay = System.currentTimeMillis();
            dDelay = false;
        }
        if (bucketSeq && slideHeight == SlideHeight.HIGH1 && System.currentTimeMillis()-depoDelay>130 || bucketSeq && slideHeight == SlideHeight.LOWER && System.currentTimeMillis()-depoDelay>100){
            bucketSeq = false;
            depoDelay = 0;
            robot.clawBigTilt.setBucket();
            robot.depoWrist.setBucket();
            robot.intakeTilt.setTransfer();
        }


        if (slideHeight == SlideHeight.MID && System.currentTimeMillis()-depoDelay>480 && dDelay) { // Depo to Specimen
            robot.claw.setSpec();
            robot.clawBigTilt.setOut();
            robot.depoWrist.setSpecimen();
            robot.depoHslide.setOut();
            robot.clawSmallTilt.setFlat();
            score = ScoreType.SPECIMEN;
            depoDelay = 0;
            dDelay = false;
        }

        // Wall pickup Sequence
        if(wallStep==1 && System.currentTimeMillis() - depoDelay > 100){
            robot.intakeTilt.setInOut();
            robot.clawSmallTilt.setWall();
            robot.clawBigTilt.setFlat();
            robot.depoWrist.setFlat();
            depoDelay = System.currentTimeMillis();
            wallStep++;
        }
        if(wallStep==2 && System.currentTimeMillis() - depoDelay > 130){
            clawOpen = false;
            robot.clawBigTilt.setWall();
            robot.depoWrist.setWall();
            depoDelay = System.currentTimeMillis();
            wallStep++;
        }
        if(wallStep==3 && System.currentTimeMillis() - depoDelay > 640){
            robot.clawSmallTilt.setWall();
            robot.claw.setOpen();
            clawOpen = true;
            wallStep=0;
            depoDelay = 0;
        }

        //slide reset seq
        if(slideHeight == SlideHeight.DOWN && System.currentTimeMillis()-resetDelay>260 && dDelay){
            robot.intakeTilt.setTransfer();
            robot.clawSmallTilt.setTransfer();
            robot.claw.setOpen();
            clawOpen = true;
            resetDelay =0;
            dDelay =false;
        }

        if(resetStep==1 && System.currentTimeMillis() - resetDelay > 230){
            robot.clawSmallTilt.setTransfer();
            robot.clawBigTilt.setTransfer();
            robot.depoWrist.setTransfer();
            robot.claw.setOpen();
            clawOpen = true;

            resetDelay = System.currentTimeMillis();
            resetStep++;
        }
        if(resetStep==2 && System.currentTimeMillis() - resetDelay > 450){
            robot.intakeTilt.setTransfer();
            slideHeight = SlideHeight.DOWN;

            resetStep=0;
            resetDelay = 0;
        }

        if(intakeDelay && System.currentTimeMillis()-outDelay>20){
            robot.hslides.moveEncoderTo(robot.hslides.hslides.getCurrentPosition()+55, 0.95f);
        }
        if(intakeDelay && System.currentTimeMillis()-outDelay>350){
            robot.trapdoor.setInit();
        }
        if(intakeDelay && System.currentTimeMillis()-outDelay>420){
            robot.intakeTilt.setOut();
            intakeDelay = false;
            manualOut = true;
            sense = true;
            outDelay =0;
            intakeMode = IntakeMode.IN;
            robot.intake.in();
        }

        if (hSlideGoBottom) {
            if (!hlimitswitch.getState()) {
                manualOut = false;
                turnConstant = 1f;
                robot.latch.setInit();
                robot.claw.setOpen();
                robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.hslides.hslides.setPower(-1f);
            } else if (hlimitswitch.getState()) {
                robot.intake.off();
                intakeMode = IntakeMode.OFF;
                robot.hslides.hslides.setPower(0);
                robot.latch.setInit();
                robot.intakeTilt.setTransfer();
                robot.clawBigTilt.setTransfer();
                robot.depoWrist.setTransfer();
                robot.clawSmallTilt.setTransfer();
                robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.hslides.hslides.resetPosition();
                clawDelay = System.currentTimeMillis();
                cDelay = true;
                hSlideGoBottom = false;
            }
        }

        /*
        if (vlimitswitch.getState()){
            robot.vSlides.vSlidesR.resetPosition();
            robot.vSlides.vSlidesL.resetPosition();
        } */

        //telemetry.addData("sensorF color", robot.sensorF.getColor());
        //telemetry.addData("time: ", temp);
        //telemetry.addData("vslides R power", robot.vSlides.getPowerR());
        //telemetry.addData("vslides L power", robot.vSlides.getPowerL());
        //telemetry.addData("vslides limit", vlimitswitch.getState());
        //telemetry.addData("vslides L pos", robot.vSlides.vSlidesL.getCurrentPosition());
        //telemetry.addData("Slide encoder current: ", robot.vSlides.vSlidesR.getCurrentPosition());
        //telemetry.addData("Position trying to reach: ", slideHeight);
        //telemetry.addData("hslides encoder: ", robot.hslides.hslides.getCurrentPosition());

    }


    public void slideBottom() { //Slide bottom
        if (!vlimitswitch.getState() && vslideGoBottom) {
            turnConstant = 1f;
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

    public void transferNow(){
        manualOut = false;
        intakeOutDelay = true;
        outakeTime = System.currentTimeMillis();
        robot.intake.in();
        intakeMode = IntakeMode.IN;
        intakeTransfer = true;
        hSlideGoBottom = true;
    }

    public static void waitFor(int milliseconds) { //Waitor Function
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < milliseconds) {
            // loop
        }
    }

}
