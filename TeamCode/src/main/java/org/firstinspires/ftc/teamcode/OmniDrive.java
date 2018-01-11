package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "OmniDriveJack")

public class OmniDrive extends God3OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor relic = null;
    private Servo SRelicRotate = null;
    private Servo SBlock = null;
    private Orientation mode = Orientation.LIFT;
    private Servo SRelicPickup = null;
    private boolean read = false;
    private DcMotor BR = null;
    private DcMotor BL = null;
    private Servo SL = null;
    private Servo SR = null;
    private Servo JS = null;
    private DcMotor lift = null;
    private ColorSensor CBL;
    private boolean gripped = false;
    private boolean lifted = false;
    private double short_drive_x;
    private boolean modeBool = false;
    private double short_drive_y;
    private ElapsedTime clock = new ElapsedTime();
    private double startTime = 0.0;
    int counter = 0;

    @Override
    public void strafe(boolean strafe) {
        FR.setDirection(strafe ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        FL.setDirection(strafe ? DcMotor.Direction.FORWARD : DcMotor.Direction.FORWARD);
        BR.setDirection(strafe ? DcMotor.Direction.REVERSE : DcMotor.Direction.REVERSE);
        BL.setDirection(strafe ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        FR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        FL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void runOpMode() throws InterruptedException {
        // Tell the driver that the Op Mode has started
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        relic = hardwareMap.get(DcMotor.class, "relic");
        CBL = hardwareMap.get(ColorSensor.class, "CBL");
        SR = hardwareMap.get(Servo.class, "SR");
        SL = hardwareMap.get(Servo.class, "SL");
        JS = hardwareMap.get(Servo.class, "JS");
        SRelicRotate = hardwareMap.get(Servo.class, "SRelicRotate");
        SRelicPickup = hardwareMap.get(Servo.class, "SRelicPickup");
        SBlock = hardwareMap.get(Servo.class, "SBlock");
        lift = hardwareMap.get(DcMotor.class, "lift");

        // Set the initial directions of the motors
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set the behaviour when motors' power is set to zero -- whether to brake
        FR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        FL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        relic.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        lift.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);

        // Set the initial position of the jewel servo
        JS.setPosition(JEWEL_SERVO_UP);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // Reset the timer to zero.
        runtime.reset();

        // Wait for the start button to be pressed on the phone.
        waitForStart();

        // Loop until the op mode is stopped.
        while (opModeIsActive()) {

            // Send the driver the positions of the relic servos.
            telemetry.addData("relicRotatePos", SRelicRotate.getPosition());
            telemetry.addData("relicPickupPos", SRelicPickup.getPosition());
            telemetry.addData("rounded pos", Math.round(SRelicPickup.getPosition() * 100.0) / 100.0);

            // Pull up the jewel arm.
            JS.setPosition(JEWEL_SERVO_UP);

            // left stick controls direction
            // right stick X controls rotation

            // Get data from the gamepad and scale it appropriately. The scale is based upon whether the right bumper is pressed.
            double scale = (gamepad1.right_bumper ? .3 : .7);
            double drive_scale = (gamepad1.right_bumper ? .3 : 1);
            double gamepad1LeftY = -gamepad1.left_stick_y * drive_scale;
            double gamepad1LeftX = gamepad1.left_stick_x * drive_scale;
            double gamepad1RightX = gamepad1.right_stick_x * scale;

            // Apply the holonomic formulas to calculate the powers of the motors
            double frontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            double frontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            double backRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            double backLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

            telemetry.addData("read", read);

            // If the joystick values are past the threshold, set the power variables to the clipped calculated power.
            // Otherwise, set them to zero.
            if (Math.abs(gamepad1LeftX) > .2 || Math.abs(gamepad1LeftY) > .2 || Math.abs(gamepad1RightX) > .2) {

                // clip the right/left values so that the values never exceed +/- 1
                frontRight = Range.clip(frontRight, -1, 1);
                frontLeft = Range.clip(frontLeft, -1, 1);
                backLeft = Range.clip(backLeft, -1, 1);
                backRight = Range.clip(backRight, -1, 1);
            } else {
                frontRight = 0;
                frontLeft = 0;
                backRight = 0;
                backLeft = 0;
            }

            // Send the power variables to the driver.
            telemetry.addData("FR", frontRight);
            telemetry.addData("FL", frontLeft);
            telemetry.addData("BR", backRight);
            telemetry.addData("BL", backLeft);

            // Set the powers of the motors to the power variables.
            FR.setPower(frontRight);
            FL.setPower(frontLeft);
            BR.setPower(backRight);
            BL.setPower(backLeft);

            // Send the relic servo's position to the driver.
            telemetry.addData("relicPos", relic.getCurrentPosition());

            // Open and close the lift servos based upon the second gamepad.
            if (gamepad2.left_trigger > .2) {
                if (SR.getPosition() != RIGHT_SERVO_OPEN) {
                    SR.setPosition(RIGHT_SERVO_OPEN);
                    SL.setPosition(LEFT_SERVO_OPEN);
                }
            } else if (gamepad2.right_trigger > .2) {
                if (SR.getPosition() != RIGHT_SERVO_FLAT) {
                    SR.setPosition(RIGHT_SERVO_FLAT);
                }
                if (SL.getPosition() != LEFT_SERVO_FLAT) {
                    SL.setPosition(LEFT_SERVO_FLAT);
                }
            } else if (gamepad2.left_bumper) {
                if (SL.getPosition() != LEFT_SERVO_OPEN) {
                    SL.setPosition(LEFT_SERVO_OPEN);
                }
            } else if (gamepad2.right_bumper) {
                if (SR.getPosition() != RIGHT_SERVO_OPEN) {
                    SR.setPosition(RIGHT_SERVO_OPEN);
                }
            } else if (gamepad2.a) {
                if (SR.getPosition() != RIGHT_SERVO_AJAR) {
                    SR.setPosition(RIGHT_SERVO_AJAR);
                }
                if (SL.getPosition() != LEFT_SERVO_AJAR) {
                    SL.setPosition(LEFT_SERVO_AJAR);
                }
            } else if (Math.abs(gamepad2.right_stick_y) > .2) {
                relic.setPower(map(gamepad2.right_stick_y, -1.0, 1.0, -.7, .7));
            } else if (gamepad2.left_bumper) {
                SL.setPosition(LEFT_SERVO_OPEN);
            } else {
                relic.setPower(0.0);
                // servo test
                SR.setPosition(RIGHT_SERVO_CLOSED);
                SL.setPosition(LEFT_SERVO_CLOSED);
            }

            // Raise or lower the lift based upon the second gamepad's D-pad.
            if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                lift.setPower(1);
                telemetry.addData("Lift", "Lowering");
            } else if (!gamepad2.dpad_up && gamepad2.dpad_down) {
                lift.setPower(-1);
                telemetry.addData("Lift", "Raising");
            } else {
                lift.setPower(0);
                telemetry.addData("Lift", "Stationary");
            }

            // Open or close the relic servos based upon the second gamepad.
            if (gamepad2.b) {
                if (!gripped) {
                    gripped = true;
                    if (Math.round(SRelicPickup.getPosition() * 100.0) / 100.0 == RELIC_PICKUP) {
                        SRelicPickup.setPosition(RELIC_DROP);
                    } else if (Math.round(SRelicPickup.getPosition() * 100.0) / 100.0 == RELIC_DROP) {
                        SRelicPickup.setPosition(RELIC_PICKUP);
                    } else {
                        SRelicPickup.setPosition(RELIC_DROP);
                    }
                }
            } else if (gamepad2.x) {
                if (!read) {
                    read = true;

                    if (Math.round(SRelicRotate.getPosition() * 100.0) / 100.0 == RELIC_FLIPDOWN) {
                        SRelicRotate.setPosition(RELIC_FLIPUP);
                    } else if (Math.round(SRelicRotate.getPosition() * 100.0) / 100.0 == RELIC_FLIPUP) {
                        SRelicRotate.setPosition(RELIC_FLIPDOWN);
                    } else {
                        SRelicRotate.setPosition(RELIC_FLIPUP);
                    }
                }
                // SRelicRotate.setPosition(RELIC_GRIPPED);
            } else {
                gripped = false;
                read = false;
            }

            // If y is pressed, toggle whether the robot is in relic or lift mode. This changes
            // which direction is "forward" to better suit the task at hand.
            if (gamepad1.y) {
                if (!modeBool) {
                    modeBool = true;
                    if (mode == Orientation.LIFT) {
                        switchToRelic();
                    } else {
                        switchToLift();
                    }
                }
            } else {
                modeBool = false;
            }

            // Update the displayed values on the driver phone.
            telemetry.update();
        }

        // When the op mode is told to stop, stop the motors.
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }

    /*
     * Scales a value to the appropriate range--used for calculating motor powers/servo positions.
     * For instance, you could use this to map 5 in the range (0,10) to 0.25 in the range (0,0.5)
     */
    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /**
     * Switch to relic orientation
     */
    public void switchToRelic() {
        SBlock.setPosition(.5);
        mode = Orientation.RELIC;
        FR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BL");
        BL = hardwareMap.get(DcMotor.class, "FL");
        SRelicRotate.setPosition(RELIC_FLIPDOWN);
        SRelicPickup.setPosition(RELIC_DROP);
    }

    /**
     * Switch to lift orientation
     */
    public void switchToLift() {
        mode = Orientation.LIFT;
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
    }

    /**
     * Represents which direction is "forward;" this can be changed
     * based upon whether the relic mechanism or lift is being used.
     */
    private static enum Orientation {
        RELIC, LIFT
    }
}
