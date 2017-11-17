package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by student on 10/19/17.
 */

public abstract class AbstractAutonomous extends God3OpMode {

    /**
     * Jewel servo down position
     */
    /**
     * The power with which to turn when knocking off the jewel.
     */
    private static final int JEWEL_TURN_TIME = 25;

    /**
     * Clock to time operations
     */
     ElapsedTime clock = new ElapsedTime();

    /**
     * Right color sensor
     */
     ColorSensor CBR;

    /**
     * Left color sensor
     */
     ColorSensor CBL;

    /**
     * Front-right Servo
     */
     DcMotor FR = null;

    /**
     * Front-left Servo
     */
     DcMotor FL = null;

    /**
     * Back-right Servo
     */
     DcMotor BR = null;

    /**
     * Back-left Servo
     */
     DcMotor BL = null;

    /**
     * Jewel Servo
     */
     Servo JS = null;

    /**
     * Left grabber servo
     */
     Servo SL = null;

    /**
     * Right grabber servo
     */
     Servo SR = null;

     DcMotor lift = null;

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
    public void runOpMode() throws InterruptedException {
        // Declare any local / helper variables here

        // Our initialization code should go here

        // Example: Map the hardware to the arm_motor variable
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables.
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        CBR = hardwareMap.get(ColorSensor.class, "CBR");
        CBL = hardwareMap.get(ColorSensor.class, "CBL");
        JS = hardwareMap.get(Servo.class, "JS");
        lift = hardwareMap.get(DcMotor.class, "lift");
        SR = hardwareMap.get(Servo.class, "SR");
        SL = hardwareMap.get(Servo.class, "SL");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        strafe(false);

        JS.setPosition(JEWEL_SERVO_UP);

        openGrabber();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            getGlyph();
            pushJewel();
        }

        // ...
    }

    /**
     * Get the preloaded glyph
     */
    private void getGlyph() {
        closeGrabber();

        double startTime = clock.milliseconds();
        while (clock.milliseconds() - startTime < 500) {
        }
        startTime = clock.milliseconds();
        while (clock.milliseconds() - startTime < 500) {
            lift.setPower(.7);
        }
        lift.setPower(0);
    }

    /**
     * Push the correct jewel
     */
    void pushJewel() {
        double power = 0.6;

        JS.setPosition(JEWEL_SERVO_DOWN);
        double startTime = clock.milliseconds();

        while (clock.milliseconds() - startTime < 2000) {
            telemetry.addData("CBR R,G,B", "(" + CBR.red() + ", " + CBR.green() + ", " + CBR.blue() + ")");
            telemetry.addData("CBL R,G,B", "(" + CBL.red() + ", " + CBL.green() + ", " + CBL.blue() + ")");
        }

        telemetry.addData("result", get_colors());

        if (!isBlue()) {
            if (get_colors() == JewelPosition.RED_JEWEL_LEFT) {
                drive(.3, 0, 0, JEWEL_TURN_TIME);
                delay(500);
                drive(-.3, 0, 0, JEWEL_TURN_TIME);
            } else if (get_colors() == JewelPosition.RED_JEWEL_RIGHT) {
                drive(-.3, 0, 0, JEWEL_TURN_TIME);
                delay(500);
                drive(.3, 0, 0, JEWEL_TURN_TIME);
            }
        } else {
            if (get_colors() == JewelPosition.RED_JEWEL_RIGHT) {
                drive(.5, 0, 0, JEWEL_TURN_TIME);
                delay(500);
                drive(-.5, 0, 0, JEWEL_TURN_TIME);
            } else if (get_colors() == JewelPosition.RED_JEWEL_LEFT) {
                drive(-.5, 0, 0, JEWEL_TURN_TIME);
                delay(500);
                drive(.5, 0, 0, JEWEL_TURN_TIME);
            }
        }
        JS.setPosition(JEWEL_SERVO_UP);
        delay(1000);
    }
    public void delay(int time) {
        double startTime = clock.milliseconds();
        while (clock.milliseconds() - startTime < time) {
        }
    }
    public void drive(double turn, double drive_x, double drive_y, int time) {
        drive(turn, drive_x, drive_y);
        delay(time);
        drive(0, 0, 0);
    }
    public void drive(double turn, double drive_x, double drive_y) {
        double leftPower;
        double rightPower;
        double startTime = clock.milliseconds();

            telemetry.addData("CBR R,G,B", "(" + CBR.red() + ", " + CBR.green() + ", " + CBR.blue() + ")");
            telemetry.addData("CBL R,G,B", "(" + CBL.red() + ", " + CBL.green() + ", " + CBL.blue() + ")");

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
        telemetry.update();
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */


    /*
    * The function currently returns the location of the Red Jewel.
    * It can be refactored later to return a color based on the
    * alliance, but this version makes no assumption about what jewel
    * the user wants to know about.
    */
    public JewelPosition get_colors() {
        if (CBR.red() > CBL.red() && CBR.blue() < CBL.blue()) {
            return JewelPosition.RED_JEWEL_RIGHT;
        } else if (CBR.red() < CBL.red() && CBR.blue() > CBL.blue()) {
            return JewelPosition.RED_JEWEL_LEFT;
        }

        if (CBR.red() > CBR.blue() && CBL.red() < CBL.blue()) {
            return JewelPosition.RED_JEWEL_RIGHT;
        } else if (CBL.red() > CBL.blue() && CBR.red() < CBR.blue()) {
            return JewelPosition.RED_JEWEL_LEFT;
        }

        return JewelPosition.JEWEL_INCONCLUSIVE;
    }

    public abstract Alliance getAlliance();

    private boolean isRed() {
        return getAlliance() == Alliance.RED;
    }

    private boolean isBlue() {
        return getAlliance() == Alliance.BLUE;
    }

    /**
     * Close the grabber
     */
    void closeGrabber() {
        if (SR.getPosition() != RIGHT_SERVO_CLOSED) {
            SR.setPosition(RIGHT_SERVO_CLOSED);
            SL.setPosition(LEFT_SERVO_CLOSED);
        }
    }

    /**
     * Open the grabber
     */
    void openGrabber() {
        SR.setPosition(RIGHT_SERVO_OPEN);
        SL.setPosition(LEFT_SERVO_OPEN);
    }
    void openGrabberFlat() {
        SR.setPosition(RIGHT_SERVO_FLAT);
        SL.setPosition(LEFT_SERVO_FLAT);
    }
    public enum Alliance {
        BLUE, RED
    }

    public enum JewelPosition {
        RED_JEWEL_LEFT, RED_JEWEL_RIGHT, JEWEL_INCONCLUSIVE
    }
}
