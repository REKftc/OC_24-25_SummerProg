package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_T;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.Button;
import overcharged.components.RobotMecanum;
import overcharged.components.colorSensor;
import overcharged.components.vSlides;



@Config
@TeleOp(name="turntable red", group="!!Teleop")
public class teleop8 extends OpMode{

    RobotMecanum robot;
    private DigitalChannel hlimitswitch;
    private DigitalChannel vlimitswitch;

    long depoDelay;
    long resetDelay;
    long clawDelay;
    long outakeTime;
    long specOutDelay;
    long manualDelay;
    long outDelay;
    ElapsedTime temp;

    double slowPower = 1;
    float turnConstant = 1f;

    int wallStep = 0;
    int specWallStep = 0;
    int resetStep = 0;
    int transferStep = 0;
    int intakeStep = 0;

    boolean intakeTransfer = true;
    boolean intakeDelay = false;
    boolean intakeOn = false;
    boolean clawOpen = true;
    boolean hangUp = false;
    boolean hslideOut = false;
    boolean vslideOut = false;
    boolean manualOut = false;
    boolean sense = false;
    boolean manualCheck = false;
    boolean canYellow = true;
    boolean hslideManualOnce = false;
    boolean onlyUsedForInCheck = false;
    boolean anotherBooleanWithOneSingularUse = false;
    boolean curSpec = false;
    boolean specOutRetract = false;

    boolean dDelay = false;
    boolean cDelay = false;
    boolean bucketSeq = false;
    boolean intakeOutDelay = false;
    boolean latch = true;
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
        temp = new ElapsedTime();
        robot.vSlides.vSlidesR.resetPosition();
        robot.vSlides.vSlidesL.resetPosition();
        robot.hslides.hslides.resetPosition();
    }

    public void loop(){
        robot.clearBulkCache();
        temp.reset();
        robot.sensorF.update();
        long timestamp = System.currentTimeMillis();
        robot.vSlides.update();

        telemetry.addData("target",robot.vSlides.getTarget());
       /* if (robot.hangLeft == null || robot.hangRight == null) {
            telemetry.addData("Error", "hangLeft or hangRight not initialized");
            telemetry.update();
            return;
        }*/

        //Driving
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

        if(gamepad1.right_bumper && Button.SLIDE_RESET.canPress(timestamp)){ //hslide out
            robot.latch.setOut();
            turnConstant = 0.55f;
            robot.hslides.moveEncoderTo(robot.hslides.SMALL_OUT,1f);
            robot.intake.in();
            intakeMode = IntakeMode.IN;
            sense = true;

            manualDelay = System.currentTimeMillis();
            hslideOut = true;
            onlyUsedForInCheck = true;
            manualCheck = true;
        }

        if (manualOut) { //manual slide control
            float slidePower = -gamepad1.right_stick_y;
            if (Math.abs(slidePower) > 0.05) {
                hslideManualOnce = true;
                robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.hslides.hslides.setPower(slidePower*0.65f);
            } else {
                if (hslideManualOnce) {
                    hslideManualOnce = false;
                    robot.hslides.hslides.setPower(0);
                }
            }
        }

        if (gamepad1.left_bumper && Button.TRANSFER.canPress(timestamp)) { //intaketilt down
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
                if(!anotherBooleanWithOneSingularUse) {
                    robot.intake.in();
                    intakeMode = IntakeMode.IN;
                } else if(robot.sensorF.getColor() == colorSensor.Color.RED){
                    robot.intake.out();
                    intakeMode = IntakeMode.OUT;
                    specOutRetract = true;
                    specOutDelay = System.currentTimeMillis();
                } else{
                    robot.intake.in();
                    intakeMode = IntakeMode.IN;
                }
                intakeTransfer = false;
            }
        }

        if(specOutRetract && System.currentTimeMillis()-specOutDelay>120){
            hSlideGoBottom = true;
            specOutDelay = 0;
            specOutRetract = false;
        }

        if (gamepad1.share && Button.INTAKEDOOR.canPress(timestamp)){ //Take yellow, or not(boolean)
            canYellow = !canYellow;
        }

        if (gamepad1.right_trigger > 0.75 && Button.INTAKE.canPress(timestamp)) { //Intake
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

        if(gamepad1.left_trigger > 0.75&& Button.INTAKEOUT.canPress(timestamp)){ //Outtake
            if(intakeMode == IntakeMode.OFF || intakeMode == IntakeMode.IN) {
                sense = false;
                robot.intake.out();
                intakeMode = IntakeMode.OUT;
            }
            else {
                robot.intake.off();
                intakeMode = IntakeMode.OFF;
            }
        }

        if(gamepad1.ps){
            anotherBooleanWithOneSingularUse = !anotherBooleanWithOneSingularUse;
            if(anotherBooleanWithOneSingularUse){
                gamepad1.setLedColor(255,0,0,1000);
            } else{
                gamepad1.setLedColor(0,0,255,1000);
            }
        }

        if (gamepad1.x || gamepad2.right_bumper) { //TODO: fast outtake
            sense = false;
            //placeholder
        }

        if(gamepad1.y && Button.TRANSFER.canPress(timestamp)){ //Force transfer for base driver
            transferNow();
        }

        if(gamepad1.b && Button.NOPOWER.canPress(timestamp)){
           /* vslideGoBottom = true;
            vlimitswitch.setState(true);
            robot.vSlides.vSlidesR.setPower(0);
            robot.vSlides.vSlidesL.setPower(0);
            robot.vSlides.vSlidesL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.vSlides.vSlidesR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
        }



        if(intakeOutDelay){ // automatic outtake after sensing right block
            intakeOutDelay = false;
            robot.intake.in();
            intakeMode = IntakeMode.IN;
            intakeStep = 0;
            intakeStep++;
            outakeTime = System.currentTimeMillis();
        } if(intakeStep == 1 && System.currentTimeMillis()-outakeTime>120){
            robot.intake.slowOut();
            intakeMode = IntakeMode.OUT;
            intakeStep++;
            outakeTime = System.currentTimeMillis();
        } if(intakeStep == 2 && System.currentTimeMillis()-outakeTime>220){
            robot.intake.in();
            intakeMode = IntakeMode.IN;
            intakeStep++;
            outakeTime = System.currentTimeMillis();
        } if(intakeStep == 3 && System.currentTimeMillis()-outakeTime>620){
            robot.intakeTilt.setTransfer();
            robot.intake.off();
            intakeMode = IntakeMode.OFF;
            intakeStep = 0;
            outakeTime = 0;
        }

        if(intakeTransfer && cDelay && System.currentTimeMillis()-clawDelay>40){ // Transfer System with claw close
            cDelay = false;
            robot.depoTilt.setTransfer();
            robot.depoHslide.setTransfer();
            transferStep = 0;
            transferStep++;
            clawDelay = System.currentTimeMillis();
        } if (transferStep == 1 & System.currentTimeMillis()-clawDelay>300){
            if(canYellow) {
                //robot.claw.setClose();
                //clawOpen = false;
            }
            transferStep++;
            clawDelay = System.currentTimeMillis();
        } if (transferStep == 2 & System.currentTimeMillis()-clawDelay>50){
            if(canYellow) {
                gamepad2.rumble(500);
                transferStep = 0;
                clawDelay = 0;
            }
        }

        if(intakeMode == IntakeMode.IN && sense && !intakeTransfer){ // block sensing
            if (robot.sensorF.getColor() == colorSensor.Color.RED){
                intakeOn = false;
                intakeTransfer = true;
                robot.intakeTilt.setHigh();
                transferNow();
                sense = false;
                hslideOut = false;
            } if (robot.sensorF.getColor() == colorSensor.Color.YELLOW && canYellow){
                intakeOn = false;
                intakeMode = IntakeMode.OFF;
                robot.intakeTilt.setHigh();
                intakeTransfer = true;
                robot.intake.off();
                transferNow();
                sense = false;
                hslideOut = false;
            } else if (robot.sensorF.getColor() == colorSensor.Color.YELLOW && !canYellow){
                intakeMode = IntakeMode.OFF;
                robot.intake.slowOut();
                intakeOn = true;
                manualOut = false;
                intakeDelay = true;
                sense = false;
                outDelay = System.currentTimeMillis();
            } if (robot.sensorF.getColor() == colorSensor.Color.BLUE){
                intakeMode = IntakeMode.OFF;
                robot.intake.slowOut();
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

        if (gamepad1.a && Button.HANGRON.canPress(timestamp)) {
            if(!hangUp) {
                robot.hangRight.upRight();
                robot.hangLeft.upLeft();
                hangUp = true;
            }
            else if(hangUp){
                robot.hangRight.downRight();
                robot.hangLeft.downLeft();
                hangUp = false;
            }
        }

        if (gamepad2.y && Button.RELEASE.canPress(timestamp)) {
                robot.hangRelease.setOut();
        }

        if(gamepad2.dpad_up && Button.HIGH1.canPress(timestamp)) { //vSlides Up to Bucket
            intakeTransfer = false;
            robot.claw.setClose();
            clawOpen = false;
            slowPower = 0.85f;
            robot.claw.setClose();
            clawDelay = System.currentTimeMillis();
            cDelay = true;
            turnConstant = 0.8f;
            slideHeight = SlideHeight.HIGH1;
        } if(cDelay && slideHeight == SlideHeight.HIGH1 && clawDelay > 100){
            cDelay= false;
            clawDelay = 0;
            robot.depoHslide.setInit();
            robot.vSlides.setUseSquID(true, vSlides.high1, 1f);
            dDelay = true;
            depoDelay = System.currentTimeMillis();
        }

        if (gamepad2.left_bumper && Button.BTN_LEVEL2.canPress(timestamp)){ // Lower Bucket
            intakeTransfer = false;

            slowPower = 0.85f;
            robot.claw.setClose();
            clawOpen = false;

            slideHeight = SlideHeight.LOWER;
            robot.vSlides.setUseSquID(true, vSlides.lower, 1f);

            robot.depoHslide.setInit();

            dDelay = true;
            depoDelay = System.currentTimeMillis();
        }

        if(gamepad2.dpad_left && Button.BTN_MID.canPress(timestamp)) { // High Specimen
            intakeTransfer = false;
            curSpec = true;

            robot.claw.setClose();
            clawOpen = false;
            clawDelay = System.currentTimeMillis();
            cDelay = true;
            slideHeight = SlideHeight.MID;
        } if(cDelay && slideHeight == SlideHeight.MID && clawDelay > 200){
            cDelay= false;
            clawDelay = 0;
            vslideOut = true;
            dDelay = true;
            robot.vSlides.setUseSquID(true, vSlides.mid, 1f);
            depoDelay = System.currentTimeMillis();
        }

        if(gamepad2.dpad_right && Button.WALL.canPress(timestamp)) { // Wall sequence
            intakeTransfer = false;
            robot.depoHslide.setInit();
            if(curSpec){
                robot.claw.setOpen();
                clawOpen = true;
                specWallStep = 0;
                depoDelay = System.currentTimeMillis();
                specWallStep++;
            }
            else {
                vslideOut = true;
                wallStep = 0;
                robot.claw.setClose();
                clawOpen = false;
                depoDelay = System.currentTimeMillis();
                wallStep++;
            }
        }

        if(gamepad2.dpad_down && Button.SLIDE_RESET.canPress(timestamp)) { // Slide reset
            robot.vSlides.setUseSquID(false, 0);
            slowPower = 1f;
            robot.depoHslide.setInit();
            curSpec = false;
            if(slideHeight == SlideHeight.DOWN || slideHeight == SlideHeight.WALL || slideHeight == SlideHeight.LOWER) {
                vslideGoBottom = true;
                robot.claw.setClose();
                robot.intakeTilt.setFlat();
                robot.depoTilt.setTransfer();

                resetDelay = System.currentTimeMillis();
                resetStep++;
            } else{
                vslideGoBottom = true;
                robot.depoTilt.setTransfer();

                slideHeight = SlideHeight.DOWN;
                resetDelay = System.currentTimeMillis();
                dDelay = true;
            }
        }

        if(gamepad2.x && Button.SLIGHT_UP.canPress(timestamp)){ // slight slides up
            if(robot.vSlides.vSlidesL.getCurrentPosition() < robot.vSlides.high1+101){
                robot.vSlides.setUseSquID(true, robot.vSlides.vSlidesL.getCurrentPosition()+50, 1f);
            }
        }

        if(gamepad2.b && Button.SLIGHT_DOWN.canPress(timestamp)){ // slight slides down
            if(robot.vSlides.vSlidesL.getCurrentPosition() > 51){
                robot.vSlides.setUseSquID(true, robot.vSlides.vSlidesL.getCurrentPosition()-50, 1f);
            }
        }

        if(vslideGoBottom){ //Reset vSlide check
            slideBottom();
        }

        if(hslideOut && System.currentTimeMillis()-manualDelay>600 && manualCheck){
            manualCheck = false;
            manualDelay = 0;
            manualOut = true;
        }

        // Bucket(High & Low) sequence
        if (slideHeight == SlideHeight.HIGH1 && System.currentTimeMillis()-depoDelay>240 &dDelay || slideHeight == SlideHeight.LOWER && System.currentTimeMillis()-depoDelay>200 &dDelay) { // Depo to Bucket
            vslideOut = true;

            robot.depoTilt.setOut();
            bucketSeq = true;
            depoDelay = System.currentTimeMillis();
            dDelay = false;
        } if (bucketSeq && slideHeight == SlideHeight.HIGH1 && System.currentTimeMillis()-depoDelay>80 || bucketSeq && slideHeight == SlideHeight.LOWER && System.currentTimeMillis()-depoDelay>60){
            bucketSeq = false;
            depoDelay = 0;
            robot.intakeTilt.setTransfer();
            robot.depoHslide.setOut();
        }


        if (slideHeight == SlideHeight.MID && System.currentTimeMillis()-depoDelay>100 && dDelay) { // Depo to Specimen
            robot.claw.setClose();
        } if(slideHeight == SlideHeight.MID && System.currentTimeMillis()-depoDelay>350 && dDelay) {
            robot.depoTilt.setSpec();
        } if(slideHeight == SlideHeight.MID && System.currentTimeMillis()-depoDelay>600 && dDelay) {
            robot.depoHslide.setOut();
            depoDelay = 0;
            dDelay = false;
        }

        // Wall pickup Sequence
        if(wallStep==1 && System.currentTimeMillis() - depoDelay > 150){
            robot.intakeTilt.setOut();
            depoDelay = System.currentTimeMillis();
            wallStep++;
        }
        if(wallStep==2 && System.currentTimeMillis() - depoDelay >180){
            robot.depoTilt.setWall();
            clawOpen = false;
            depoDelay = System.currentTimeMillis();
            wallStep++;
        } if(wallStep==3 && System.currentTimeMillis() - depoDelay > 650){
            robot.claw.setOpen();
            clawOpen = true;
            wallStep=0;
            depoDelay = 0;
        }

        // Wall pickup Sequence from Specimen score
        if(specWallStep==1 && System.currentTimeMillis() - depoDelay > 220){
            robot.depoHslide.setInit();
            robot.depoTilt.setWall();
            depoDelay = System.currentTimeMillis();
            specWallStep++;
        }
        if(specWallStep==2 && System.currentTimeMillis() - depoDelay >300){
            slideHeight = SlideHeight.WALL;
            robot.vSlides.setUseSquID(true, vSlides.wall, 1f);
            vslideOut = true;
            specWallStep=0;
            depoDelay = 0;
        }

        //slide reset seq
        if(slideHeight == SlideHeight.DOWN && System.currentTimeMillis()-resetDelay>160 && dDelay){
            intakeTransfer = true;
            robot.intakeTilt.setTransfer();
            robot.depoTilt.setTransfer();
            robot.depoHslide.setTransfer();
            robot.claw.setOpen();
            clawOpen = true;
            resetDelay =0;
            dDelay =false;
        } if(resetStep==1 && System.currentTimeMillis() - resetDelay > 200){
            robot.depoTilt.setTransfer();
            robot.claw.setOpen();
            clawOpen = true;
            intakeTransfer = true;

            resetStep=0;
            resetDelay = 0;
        }

        if(intakeDelay && System.currentTimeMillis()-outDelay>600){
            intakeDelay = false;
            manualOut = true;
            sense = true;
            outDelay =0;
            intakeMode = IntakeMode.IN;
            robot.intake.in();
        }

        if (hSlideGoBottom) { // hslides reeset
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
                robot.depoHslide.setTransfer();
                robot.intakeTilt.setTransfer();
                robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.hslides.hslides.resetPosition();
                clawDelay = System.currentTimeMillis();
                cDelay = true;
                hSlideGoBottom = false;
            }
        }


        telemetry.addData("lag: ", temp);
        telemetry.addData("Can Yellow?: ", canYellow);
        telemetry.addData("Slide encoder current R: ", robot.vSlides.vSlidesR.getCurrentPosition());
        telemetry.addData("Slide encoder current L: ", robot.vSlides.vSlidesL.getCurrentPosition());
        //telemetry.addData("hslides encoder: ", robot.hslides.hslides.getCurrentPosition());
        //telemetry.addData("sensorF color", robot.sensorF.getColor());
        //telemetry.addData("vlimit: ", vlimitswitch.getState());
        //telemetry.addData("hlimit: ", hlimitswitch.getState());
    }

    public void slideBottom() { //vSlide bottom
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
        intakeMode = IntakeMode.IN;
        intakeTransfer = true;
        if(onlyUsedForInCheck) {
            hSlideGoBottom = true;
            onlyUsedForInCheck = false;
        }
    }
}