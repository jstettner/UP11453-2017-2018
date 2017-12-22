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
    private String mode = "lift";
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
    BNO055IMU imu = null;

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
        telemetry.addData("Status", "Initialized");
        initGyro();
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

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
        FR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        FL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        relic.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
//        strafe(false);
        JS.setPosition(JEWEL_SERVO_UP);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        double startingAngle = 0;
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        runtime.reset();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("relicRotatePos", SRelicRotate.getPosition());
            telemetry.addData("relicPickupPos", SRelicPickup.getPosition());
            telemetry.addData("rounded pos", Math.round(SRelicPickup.getPosition() * 100.0) / 100.0);
            JS.setPosition(JEWEL_SERVO_UP);
            // left stick controls direction
            // right stick X controls rotation

            double scale = (gamepad1.right_bumper ? .3 : .7);
            double drive_scale = (gamepad1.right_bumper ? .3 : 1);

            double gamepad1LeftY = -gamepad1.left_stick_y * drive_scale;
            double gamepad1LeftX = gamepad1.left_stick_x * drive_scale;
            double gamepad1RightX = gamepad1.right_stick_x * scale;
            double frontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            double frontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            double backRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            double backLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            telemetry.addData("read", read);
            if (Math.abs(gamepad1LeftX) > .2 || Math.abs(gamepad1LeftY) > .2 || Math.abs(gamepad1RightX) > .2) {
                // holonomic formulas

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
            telemetry.addData("FR", frontRight);
            telemetry.addData("FL", frontLeft);
            telemetry.addData("BR", backRight);
            telemetry.addData("BL", backLeft);
            double tolerance = .1;
            FR.setPower(frontRight);
            FL.setPower(frontLeft);
            BR.setPower(backRight);
            BL.setPower(backLeft);

            // Setup a variable for each drive wheel to save power level for telemetry
//        double leftPower;
//        double rightPower;
//        double scale = (gamepad1.right_bumper ? .3 : .7);
//        double drive_scale = (gamepad1.right_bumper ? .3 : 1);
//
//        telemetry.addData("CBR R,G,B", "(" + CBR.red() + ", " + CBR.green() + ", " + CBR.blue() + ")");
//        telemetry.addData("CBL R,G,B", "(" + CBL.red() + ", " + CBL.green() + ", " + CBL.blue() + ")");
//
//        // Choose to drive using either Tank Mode, or POV Mode
//        // Comment out the method that's not used.  The default below is POV.
//
//        // POV Mode uses left stick to go forward, and right stick to turn.
//        // - This uses basic math to combine motions and is easier to drive straight.
//        double drive_y = -gamepad1.left_stick_y * drive_scale;
//        double drive_x = gamepad1.left_stick_x * drive_scale;
//        telemetry.addData("drive_y", drive_y);
//        telemetry.addData("drive_x", drive_x);
//        double turn = gamepad1.right_stick_x * scale;
//        telemetry.addData("turn", turn);
//
//        if (Math.abs(turn) < .2) {
//            turn = 0;
//        }
//
//        if (Math.abs(drive_y) > .2) {
//            telemetry.addData("Status", "Driving");
//            strafe(false);
//
//            leftPower = Range.clip(drive_y + turn, -1.0, 1.0);
//            rightPower = Range.clip(drive_y - turn, -1.0, 1.0);
//
//            FL.setPower(leftPower);
//            BL.setPower(leftPower);
//            FR.setPower(rightPower);
//            BR.setPower(rightPower);
//        } else if (Math.abs(drive_x) > .2) {
//            telemetry.addData("Status", "Strafing");
//            strafe(true);
//
//            leftPower = Range.clip(drive_x + turn, -1.0, 1.0);
//            rightPower = Range.clip(drive_x - turn, -1.0, 1.0);
//
//            FL.setPower(leftPower);
//            BL.setPower(rightPower);
//            FR.setPower(leftPower);
//            BR.setPower(rightPower);
//        } else {
//            telemetry.addData("Status", "Turning");
//            strafe(false);
//
//            leftPower = Range.clip(turn, -1.0, 1.0);
//            rightPower = Range.clip(-turn, -1.0, 1.0);
//
//            FL.setPower(leftPower);
//            BL.setPower(leftPower);
//            FR.setPower(rightPower);
//            BR.setPower(rightPower);
//        }
           /* telemetry.addData("read", read);
            telemetry.addData("gripped", gripped);
            telemetry.addData("relic pos: ", SRelicRotate.getPosition());
            telemetry.addData("relic pos: ", SRelicPickup.getPosition());*/
            telemetry.addData("relicPos", relic.getCurrentPosition());

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
            } else if (gamepad2.y) {
                //SRelicPickup.setPosition(RELIC_PICKUP);
            } else if (gamepad2.left_bumper) {
                SL.setPosition(LEFT_SERVO_OPEN);
            } else {
                gripped = false;
                read = false;
                relic.setPower(0.0);
                // servo test
                SR.setPosition(RIGHT_SERVO_CLOSED);
                SL.setPosition(LEFT_SERVO_CLOSED);
            }

            // Raise or lower the lift

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
            if (gamepad2.b) {
                //      SRelicPickup.setPosition(RELIC_DROP);
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
                   /* relic.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (relic.getTargetPosition() > -6000 + 25) {
                        relic.setTargetPosition(-6000);
                        relic.setPower(.8);
                    }
                    relic.setPower(0);*/
                    //   relic.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if (Math.round(SRelicRotate.getPosition() * 100.0) / 100.0 == RELIC_FLIPDOWN) {
                        SRelicRotate.setPosition(RELIC_FLIPUP);
                    } else if (Math.round(SRelicRotate.getPosition() * 100.0) / 100.0 == RELIC_FLIPUP) {
                        SRelicRotate.setPosition(RELIC_FLIPDOWN);
                    } else {
                        SRelicRotate.setPosition(RELIC_FLIPUP);
                    }
                }
                // SRelicRotate.setPosition(RELIC_GRIPPED);
            }
            if (gamepad1.y) {
                if (!modeBool) {
                    modeBool = true;
                    if (mode == "lift") {
                        switchToRelic();
                    } else {
                        switchToLift();
                    }
                }
                // SRelicRotate.setPosition(RELIC_GRIPPED);
            } else {
                modeBool = false;
            }

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            telemetry.update();
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        }
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }
    public void turn(double power, double angle) {
        telemetry.addData("test", "test");
        double startingAngle = angle();
        power = Math.abs(power);
        if (angle > 0) {
            while (getAngleDiff(startingAngle, angle()) < angle) {
                telemetry.addData("not working", "plz");
                telemetry.addData("angleDiff", getAngleDiff(startingAngle, angle()));
                telemetry.addData("startingAngle", startingAngle);
                if (angle() - getAngleDiff(startingAngle, angle()) < 20.0) {
                    drive(.2, 0, 0, 100);
                } else {
                    drive(power, 0, 0);
                }
            }
        } else {
            while (getAngleDiff(startingAngle, angle()) < Math.abs(angle)) {
                drive(-power, 0, 0, 0);
            }
        }
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    public void drive(double turn, double drive_x, double drive_y, double time) {
        double leftPower;
        double rightPower;
        double startTime = clock.milliseconds();

        while (clock.milliseconds() - startTime < time) {
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
        }
    }
    public void switchToRelic() {
        SBlock.setPosition(.5);
        mode = "relic";
        FR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BL");
        BL = hardwareMap.get(DcMotor.class, "FL");
        SRelicRotate.setPosition(RELIC_FLIPDOWN);
        SRelicPickup.setPosition(RELIC_DROP);
    }
    public void switchToLift() {
       // SBlock.setPosition(.9);
        mode = "lift";
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
    }
    double angle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }
    public void drive(double turn, double drive_x, double drive_y) {
        double leftPower;
        double rightPower;
        double startTime = clock.milliseconds();

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
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */

    public void initGyro() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }
}
