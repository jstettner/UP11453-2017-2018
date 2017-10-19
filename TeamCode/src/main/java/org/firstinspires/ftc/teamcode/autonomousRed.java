package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "auto1")

public class autonomousRed extends OpMode {
    // Declare OpMode members.
    ElapsedTime clock = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BR = null;
    private DcMotor BL = null;
    private Servo SL = null;
    private Servo SR = null;
    private Servo JS = null;
    private DcMotor lift = null;
    ColorSensor CBR;
    ColorSensor CBL;
    double JSdown = .6;
    double JSup = .2;

    public enum Alliance {
        BLUE, RED
    }

    public enum Jewel_Position {
        RED_JEWEL_LEFT, RED_JEWEL_RIGHT, BLUE_JEWEL_LEFT, BLUE_JEWEL_RIGHT, JEWEL_INCONCLUSIVE
    }

    public void strafe(boolean strafe) {
        FR.setDirection(strafe ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        FL.setDirection(strafe ? DcMotor.Direction.FORWARD : DcMotor.Direction.FORWARD);
        BR.setDirection(strafe ? DcMotor.Direction.REVERSE : DcMotor.Direction.REVERSE);
        BL.setDirection(strafe ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        CBR = hardwareMap.get(ColorSensor.class, "CBR");
        CBL = hardwareMap.get(ColorSensor.class, "CBL");
        SR = hardwareMap.get(Servo.class, "SR");
        SL = hardwareMap.get(Servo.class, "SL");
        JS = hardwareMap.get(Servo.class, "JS");
        lift = hardwareMap.get(DcMotor.class, "lift");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        strafe(false);

        JS.setPosition(JSup);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        double drive_y = 0;
        double drive_x = 0;
        double turn = 0;
        double power = 0.6;

        JS.setPosition(JSdown);
        double startTime = clock.milliseconds();

        while (clock.milliseconds() - startTime < 2000) {
            telemetry.addData("CBR R,G,B","("+CBR.red()+", "+CBR.green()+", "+CBR.blue()+")");
            telemetry.addData("CBL R,G,B","("+CBL.red()+", "+CBL.green()+", "+CBL.blue()+")");
        }

        telemetry.addData("result",get_colors());
        if(get_colors() == 1) {
            drive(power, 0, 0, 500);
        } else if (get_colors() == 2) {
            drive(-power, 0, 0, 500);
        }
    }

    /*
   * The function currently returns the location of the Red Jewel.
   * It can be refactored later to return a color based on the
   * alliance, but this version makes no assumption about what jewel
   * the user wants to know about.
   */
    public Jewel_Position get_colors() {
        if(CBR.red() > CBL.red() && CBR.blue() < CBL.blue()) {
            return Jewel_Position.RED_JEWEL_RIGHT;
        } else if(CBR.red() < CBL.red() && CBR.blue() > CBL.blue()) {
            return Jewel_Position.RED_JEWEL_LEFT;
        }

        if(CBR.red() > CBR.blue() && CBL.red() < CBL.blue()) {
            return Jewel_Position.RED_JEWEL_RIGHT;
        } else if(CBL.red() > CBL.blue() && CBR.red() < CBR.blue()) {
            return Jewel_Position.RED_JEWEL_LEFT;
        }

        return Jewel_Position.JEWEL_INCONCLUSIVE;
    }

    public void drive(double turn, double drive_x, double drive_y, double time) {
        double leftPower;
        double rightPower;
        double startTime = clock.milliseconds();

        while (clock.milliseconds() - startTime < 500) {
            telemetry.addData("CBR R,G,B","("+CBR.red()+", "+CBR.green()+", "+CBR.blue()+")");
            telemetry.addData("CBL R,G,B","("+CBL.red()+", "+CBL.green()+", "+CBL.blue()+")");

            if (Math.abs(turn) < .2) {
                turn = 0;
            }

            if (Math.abs(drive_y) > .2) {
                telemetry.addData("Status", "Driving");
                strafe(false);

                leftPower = Range.clip(drive_y + turn, -1.0, 1.0);
                rightPower = Range.clip(drive_y - turn, -1.0, 1.0);

                FL.setPower(leftPower);
                BL.setPower(leftPower);
                FR.setPower(rightPower);
                BR.setPower(rightPower);
            } else if (Math.abs(drive_x) > .2) {
                telemetry.addData("Status", "Strafing");
                strafe(true);

                leftPower = Range.clip(drive_x + turn, -1.0, 1.0);
                rightPower = Range.clip(drive_x - turn, -1.0, 1.0);

                FL.setPower(leftPower);
                BL.setPower(rightPower);
                FR.setPower(leftPower);
                BR.setPower(rightPower);
            } else {
                telemetry.addData("Status", "Turning");
                strafe(false);

                leftPower = Range.clip(turn, -1.0, 1.0);
                rightPower = Range.clip(-turn, -1.0, 1.0);

                FL.setPower(leftPower);
                BL.setPower(leftPower);
                FR.setPower(rightPower);
                BR.setPower(rightPower);
            }
            if (gamepad2.left_trigger > .2) {
                if (SR.getPosition() != 0) {
                    SR.setPosition(0);
                    SL.setPosition(.8);
                }
            } else {
                // servo test
                SR.setPosition(.9);
                SL.setPosition(.2);
            }
            if (gamepad1.left_trigger > .2) {
                if (JS.getPosition() != .55) {
                    //    SJ.setPosition(.55);
                }
            } else {
                // SJ.setPosition(.3);
            }
        }
        telemetry.update();
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
