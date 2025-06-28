package overcharged.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import overcharged.components.Button;
import overcharged.components.MecanumDrive;
import overcharged.components.OcMotorEx;
import overcharged.components.OcServo;
import overcharged.components.RobotMecanum;

/**
 * Overcharged Team #12599 Tester
 * This tester program has 16 separate tests.
 */
@Config
@TeleOp(name="Tester", group="Test")
public class
Tester
        extends LinearOpMode {

    ///Overcharged Autonomous Robot class
    private RobotMecanum robot;
    ///Overcharged Autonomous Tank Drive class
    MecanumDrive drive;
    //OcSwitch slideSwitch;
    //private TankDriveLinear drive;
    private DigitalChannel vlimitswitch;
    private DigitalChannel hlimitswitch;
    /**
     * Counter of servos in servo test
     */
    private int servoTestCounter = 0;

    private final List<OcServo> servos = new ArrayList<>();

    /**
     * Counter for servos for servo calibrate test
     */
    private int servoCalibrateCounter = 0;

    private final static int MIN_SERVO_TICK = 1;
    private final static DecimalFormat integerFormatter = new DecimalFormat("######");
    private final static DecimalFormat decimalFormatter = new DecimalFormat("###.##");

    /**
     * Enumeration for the different tests
     */
    public enum ETest {
        NONE,
        SWITCH,
        ENCODER,
        MOTOR,
        DRIVE,
        SERVO_CALIBRATE,
        DEPO_CALIBRATE,
        SERVO,
        //GYRO,
        //LED,
        HSLIDE,
        VSLIDE,
        SENSOR,

        ;

        private static int numberTests = 0;

        /**
         * Get the test according to number
         * @param ordinal the test number
         * @return the test according to the number given
         */
        public static ETest getTest(int ordinal)
        {
            for (ETest e : values()) {
                if (e.ordinal() == ordinal) {
                    return e;
                }
            }

            return NONE;
        }

        /**
         * Get the number of tests
         * @return the number of tests
         */
        public static int getNumberTests() {
            if (numberTests == 0) {
                for (ETest ignored : values()) {
                    numberTests++;
                }
            }
            return numberTests;
        }
    }

    /**
     * Function for running tests
     */
    @Override
    public void runOpMode() {
        //Initialization
        robot = new RobotMecanum(this, true, false);
        vlimitswitch = hardwareMap.get(DigitalChannel.class, "vlimitswitch");
        hlimitswitch = hardwareMap.get(DigitalChannel.class, "hlimitswitch");
        drive = robot.getDrive();
        robot.vSlides.vSlidesR.resetPosition();
        robot.vSlides.vSlidesL.resetPosition();
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        OcServo intakeTilt = robot.intakeTilt.intakeTilt;
        OcServo latch = robot.latch.latch;
        OcServo claw = robot.claw.claw;
        OcServo depoTiltL = robot.depoTilt.depoTiltL;
        OcServo depoTiltR = robot.depoTilt.depoTiltR;
        OcServo depoHslide = robot.depoHslide.depoHslide;
        servos.add(intakeTilt);
        servos.add(latch);
        servos.add(claw);
        servos.add(depoTiltL);
        servos.add(depoTiltR);
        servos.add(depoHslide);

        int testCounter = 0;
        ///Set current test to NONE
        ETest currentTest = ETest.NONE;

        telemetry.addData("Waiting", "Tester");
        telemetry.update();
        ///Waiting for start to be pressed
        waitForStart();

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            ///Choosing the desired test
            if(gamepad1.right_trigger > 0.9 && Button.BTN_RIGHT.canPress(timeStamp)) {
                testCounter++;
                if(testCounter >= ETest.getNumberTests()){
                    testCounter = 0;
                }
                currentTest = ETest.getTest(testCounter);
            } else if(gamepad1.left_trigger > 0.9 && Button.BTN_LEFT.canPress(timeStamp)) {
                testCounter--;
                if(testCounter < 0){
                    testCounter = ETest.getNumberTests() - 1;
                }
                currentTest = ETest.getTest(testCounter);
            }

            telemetry.addData("Test", currentTest);
            telemetry.addData("Select", "Next:RightTrigger Prev:LeftTrigger");
            telemetry.addData("Run", "Start");

            ///Loop tests
            if (gamepad1.start && Button.BTN_DISABLE.canPress(timeStamp)) {
                switch(currentTest) {
                    case SERVO_CALIBRATE:
                        servoCalibrate(servos);
                        break;
                    case NONE:
                    default:
                        break;
                }
            }

            telemetry.update();
            idle();
        }
    }

    /**
     * Calibration of servos in robot
     * @param servoCalibrateList servos to be tested
     */
    private void servoCalibrate(List<OcServo> servoCalibrateList) {
        int posJoy1 = (int)(servoCalibrateList.get(servoCalibrateCounter).getPosition());

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            ///Choose a servo for calibration
            if(gamepad1.right_trigger > 0.9 && Button.BTN_RIGHT.canPress(timeStamp)) {
                servoCalibrateCounter++;
                if(servoCalibrateCounter >= servoCalibrateList.size()){
                    servoCalibrateCounter = 0;
                }
                posJoy1 = (int)(servoCalibrateList.get(servoCalibrateCounter).getPosition());
            } else if(gamepad1.left_trigger > 0.9 && Button.BTN_LEFT.canPress(timeStamp)) {
                servoCalibrateCounter--;
                if(servoCalibrateCounter < 0){
                    servoCalibrateCounter = servoCalibrateList.size() - 1;
                }
                posJoy1 = (int)(servoCalibrateList.get(servoCalibrateCounter).getPosition());
            }
            else if (gamepad1.left_stick_button && Button.BTN_DISABLE.canPress(timeStamp)) {
                return;
            }

            ///Change servo position for calibration
            if (gamepad1.x && Button.BTN_RIGHT.canPress4Short(timeStamp)) {
                posJoy1 -= MIN_SERVO_TICK;
            } else if (gamepad1.b && Button.BTN_RIGHT.canPress4Short(timeStamp)) {
                posJoy1 += MIN_SERVO_TICK;
            } else if (gamepad1.y && Button.BTN_RIGHT.canPress(timeStamp)) {
                posJoy1 = 255;
            } else if (gamepad1.a && Button.BTN_RIGHT.canPress(timeStamp)) {
                posJoy1 = 0;
            } else if (gamepad1.right_stick_button && Button.BTN_RIGHT.canPress(timeStamp)) {
                posJoy1 = 128;
            }
            posJoy1 = Range.clip(posJoy1, 0, 255);
            servoCalibrateList.get(servoCalibrateCounter).setPosition(posJoy1);

            telemetry.addData("Test", "ServoCalibrate");
            telemetry.addData("Adjust", "+:B -:X Max:Y Min:A Mid:RStick");
            telemetry.addData("Position", integerFormatter.format(posJoy1));
            telemetry.addData("Servo", servoCalibrateList.get(servoCalibrateCounter));
            telemetry.addData("Select", "Next: RightTrigger Prev: LeftTrigger");
            telemetry.addData("Back", "LeftStick");

            telemetry.update();
            idle();
        }
    }
}