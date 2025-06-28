/*
Hey! This is an example teleop with everything to get you started.

At the start of the file, we import everything we need.
For a teleop file, we import the components(servos, motors), some control commands, and other functions to aid our code

Then we declare our file as an OpMode, which is basically a looping, runnable file on the driver hub.
We can name it and declare it as a teleop file here

The Opmode is split into different modes, init, start, init_loop, loop, etc.
init is used for initializing everything needed at the start. Loop will handle all of the commands done duringc ode.

Extra functions can be written in the file to aid with certain actions
*/


// Here are the imports!
package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_T;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import overcharged.components.Button;
import overcharged.components.RobotMecanum;
import overcharged.components.vSlides;

// here is the start of the teleop
@TeleOp(name="example teleop", group="teleop")
public class exampleTeleOp extends OpMode {

    RobotMecanum robot;

    boolean clawOpen = true;

    public void init(){ //init stuff goes here
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new RobotMecanum(this, false, false);
        robot.setBulkReadManual();
    }

    public void loop(){ //actions goes here - robot movement, claw opening, etc.
        robot.clearBulkCache();
        long timestamp = System.currentTimeMillis();

        //Driving - these lines controls the wheels!
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = ((y + x + rx) / denominator);
        double backLeftPower = ((y - x + rx) / denominator);
        double frontRightPower = ((y - x - rx) / denominator);
        double backRightPower = ((y + x - rx) / denominator);

        robot.driveLeftFront.setPower(frontLeftPower);
        robot.driveLeftBack.setPower(backLeftPower);
        robot.driveRightFront.setPower(frontRightPower);
        robot.driveRightBack.setPower(backRightPower);

        // A simple claw closing and opening code
        if (gamepad1.a && Button.CLAW.canPress(timestamp)) { // example if statement in loop to control gamepad option
            if(!clawOpen) {
                robot.claw.setOpen();
                clawOpen = true;
            }
            else if(clawOpen){
                robot.claw.setClose();
                clawOpen = false;
            }
        }

    }
}
