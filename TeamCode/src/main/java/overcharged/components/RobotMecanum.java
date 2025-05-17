package overcharged.components;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

import overcharged.config.RobotConstants;

/**
 * Overcharged Team #12599
 * Robot definition for Mecanum wheel robot
 */
public class RobotMecanum {
    protected Telemetry telemetry;

    ///Drive components
    public OcMotor driveLeftFront;
    public OcMotor driveLeftBack;
    public OcMotor driveRightFront;
    public OcMotor driveRightBack;


    public MecanumDrive drive;

    public Intake intake;
    public hang hang;
    public latch latch;
    public trapdoor trapdoor;
    public vSlides vSlides;
    public depoTilt depoTilt;
    public claw claw;
    public depoHslide depoHslide;
    public hslides hslides;
    public intakeTilt intakeTilt;
    public colorSensor sensorF;
    public pto pto;
    public smallHang smallHang;

    public final List<OcServo> servos = new ArrayList<>();
    public List<LynxModule> allHubs;

    /**
     * initialize the robot
     * initialize all the hardware components used in our robot
     * @param op opMode to run
     * @param isAutonomous if autonomous
     */
    public RobotMecanum(OpMode op, boolean isAutonomous, boolean roadrunner) {
        String missing = "";
        ///report the number of missing components
        int numberMissing = 0;
        HardwareMap hardwareMap = op.hardwareMap;
        this.telemetry = op.telemetry;

        hardwareMap.logDevices();

        allHubs = hardwareMap.getAll(LynxModule.class);

        if(!roadrunner) {
            RobotLog.ii(RobotConstants.TAG_R, "Initializing motors");
            ///Initialize Motors
            OcMotor driveLeftFront = null;
            try {
                driveLeftFront = new OcMotor(hardwareMap,
                        "driveLF",
                        DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } catch (Exception e) {
                RobotLog.ee(RobotConstants.TAG_R, "missing: driveLF " + e.getMessage());
                missing = missing + "driveLF";
                numberMissing++;
            }
            this.driveLeftFront = driveLeftFront;

            OcMotor driveLeftBack = null;
            try {
                driveLeftBack = new OcMotor(hardwareMap,
                        "driveLB",
                        DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } catch (Exception e) {
                RobotLog.ee(RobotConstants.TAG_R, "missing: driveLB " + e.getMessage());
                missing = missing + ", driveLB";
                numberMissing++;
            }
            this.driveLeftBack = driveLeftBack;

            OcMotor driveRightFront = null;
            try {
                driveRightFront = new OcMotor(hardwareMap,
                        "driveRF",
                        DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } catch (Exception e) {
                RobotLog.ee(RobotConstants.TAG_R, "missing: driveRF " + e.getMessage());
                missing = missing + ", driveRF";
                numberMissing++;
            }
            this.driveRightFront = driveRightFront;

            OcMotor driveRightBack = null;
            try {
                driveRightBack = new OcMotor(hardwareMap,
                        "driveRB",
                        DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } catch (Exception e) {
                RobotLog.ee(RobotConstants.TAG_R, "missing: driveRB " + e.getMessage());
                missing = missing + ", driveRB";
                numberMissing++;
            }
            this.driveRightBack = driveRightBack;

            this.drive = createDrive();
        }
        try {
            hslides = new hslides(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: hslides " + e.getMessage());
            missing = missing + ", hSlides";
            numberMissing++;
        }
        try {
            vSlides = new vSlides(hardwareMap);
        } catch (Exception e){
            RobotLog.ee(RobotConstants.TAG_R,  "missing: vslides " + e.getMessage());
            missing = missing + ", vslides";
            numberMissing++;
        }
        try {
            intake = new Intake(hardwareMap);
        } catch (Exception e){
            RobotLog.ee(RobotConstants.TAG_R,  "missing: intake " + e.getMessage());
            missing = missing + ", intake";
            numberMissing++;
        }
        try {
            intakeTilt = new intakeTilt(hardwareMap);
        } catch (Exception e){
            RobotLog.ee(RobotConstants.TAG_R,  "missing: IntakeBigTilt " + e.getMessage());
            missing = missing + ", IntakeBigTilt";
            numberMissing++;
        }
        try {
            depoTilt = new depoTilt(hardwareMap);
        } catch (Exception e){
            RobotLog.ee(RobotConstants.TAG_R,  "missing: clawBigTilt " + e.getMessage());
            missing = missing + ", clawBigTilt";
            numberMissing++;
        }
        try {
            claw = new claw(hardwareMap);
        } catch (Exception e){
            RobotLog.ee(RobotConstants.TAG_R,  "missing: claw " + e.getMessage());
            missing = missing + ", claw";
            numberMissing++;
        }
        try {
            depoHslide = new depoHslide(hardwareMap);
        } catch (Exception e){
            RobotLog.ee(RobotConstants.TAG_R,  "missing: depohslides " + e.getMessage());
            missing = missing + ", depohslides";
            numberMissing++;
        }
        try {
            hang = new hang(hardwareMap);
        } catch (Exception e){
            RobotLog.ee(RobotConstants.TAG_R,  "missing: hang " + e.getMessage());
            missing = missing + ", hang";
            numberMissing++;
        }
        try {
            latch = new latch(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: latch " + e.getMessage());
            missing = missing + ", latch";
            numberMissing++;
        }
        try {
            trapdoor = new trapdoor(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: trapdoor " + e.getMessage());
            missing = missing + ", trapdoor";
            numberMissing++;
        }
        try {
            pto = new pto(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: pto " + e.getMessage());
            missing = missing + ", pto";
            numberMissing++;
        }
        try {
            smallHang = new smallHang(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: smallHang " + e.getMessage());
            missing = missing + ", smallHang";
            numberMissing++;
        }

        try {
            sensorF = new colorSensor(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: sensor " + e.getMessage());
            missing = missing + ", latch";
            numberMissing++;
        }
    }



    /**
     * Robot and sensor shutdown
     */
    public void close ()
    {
        if (this.drive != null) {
            this.drive.stop();
        } else {
            if (this.driveLeftFront != null) this.driveLeftFront.setPower(0f);
            if (this.driveLeftBack != null) this.driveLeftBack.setPower(0f);
            if (this.driveRightFront != null) this.driveRightFront.setPower(0f);
            if (this.driveRightBack != null) this.driveRightBack.setPower(0f);
        }

    }

    /**
     * Initialize the underlying drive motor class (only for non-Roadrunner)
     * @return Drive
     */
    protected MecanumDrive createDrive () {
        return new MecanumDrive(
                driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack
        );
    }

    /**
     * return the Mecanum tank drive instance
     * @return MecanumDrive
     */
    public MecanumDrive getDrive () {
        return (MecanumDrive) this.drive;
    }

    /**
     * set the bulk read mode.  Default to OFF
     * @param mode bulk read mode
     */
    public void setBulkReadMode(LynxModule.BulkCachingMode mode) {
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(mode);
        }
    }

    /**
     * set the bulk read mode to manual.  You must call #clearBulkCache() at the
     * beginning of the loop
     * @see #clearBulkCache()
     */
    public void setBulkReadManual() {
        setBulkReadMode(LynxModule.BulkCachingMode.MANUAL);
    }

    /**
     * Will run one bulk read per cycle,
     * even as frontLeftMotor.getPosition() is called twice
     * because the caches are being handled manually and cleared
     * once a loop
     */

    public void setBulkReadAuto() {
        setBulkReadMode(LynxModule.BulkCachingMode.AUTO);
    }

    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}