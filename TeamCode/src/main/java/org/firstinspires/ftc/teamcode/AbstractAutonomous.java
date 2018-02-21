package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by student on 10/19/17.
 */

public abstract class AbstractAutonomous extends God3OpMode {
    BNO055IMU imu = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    /**
     * Jewel servo down position
     */
    /**
     * The power with which to turn when knocking off the jewel.
     */
    private static final int JEWEL_TURN_TIME = 125;
    double counter = 0;
    /**
     * Clock to time operations
     */
    ElapsedTime clock = new ElapsedTime();

    /**
     * Right color sensor
     */

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
    AnalogInput ultrasonicLeft;
    AnalogInput ultrasonicBack;


    /**
     * Vuforia variables
     */

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters vufParameters = null;
    VuforiaTrackables relicTrackables = null;
    VuforiaTrackable relicTemplate = null;


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
        CBL = hardwareMap.get(ColorSensor.class, "CBL");
        JS = hardwareMap.get(Servo.class, "JS");
        lift = hardwareMap.get(DcMotor.class, "lift");
        SR = hardwareMap.get(Servo.class, "SR");
        SL = hardwareMap.get(Servo.class, "SL");
        ultrasonicLeft = hardwareMap.get(AnalogInput.class, "ultrasonicLeft");

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
            //    getGlyph();
            pushJewel();
            break;
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
        initGyro();
        telemetry.addData("jewel time", JEWEL_TURN_TIME);
        telemetry.update();
        delay(3000);
        JS.setPosition(JEWEL_SERVO_DOWN);
        double startTime = clock.milliseconds();

        while (clock.milliseconds() - startTime < 2000) {
            telemetry.addData("CBL R,G,B", "(" + CBL.red() + ", " + CBL.green() + ", " + CBL.blue() + ")");
            telemetry.update();
        }

        telemetry.addData("result", get_colors());

        if (isRed()) {
            if (get_colors() == JewelPosition.RED_JEWEL_LEFT) {
                turn(.15, 15);
                JS.setPosition(JEWEL_SERVO_UP);
                delay(1500);
                turn(-.15, 14.5);
            } else if (get_colors() == JewelPosition.RED_JEWEL_RIGHT) {
                turn(-.15, 14.5);
                JS.setPosition(JEWEL_SERVO_UP);
                delay(1500);
                turn(.15, 15);
            } else {
                JS.setPosition(JEWEL_SERVO_UP);
                delay(1500);
            }
        } else {
            if (get_colors() == JewelPosition.RED_JEWEL_RIGHT) {
                turn(.15, 15);
                JS.setPosition(JEWEL_SERVO_UP);
                delay(1500);
                turn(-.15, 14.5);
            } else if (get_colors() == JewelPosition.RED_JEWEL_LEFT) {
                turn(-.15, 14.5);
                ;
                JS.setPosition(JEWEL_SERVO_UP);
                delay(1500);
                turn(.15, 15);
            } else {
                JS.setPosition(JEWEL_SERVO_UP);
                delay(1500);
            }
        }
    }

    public void delay(int time) {
        telemetry.addData("delay", "started delay");
        clock.reset();
        while (time > clock.milliseconds()) {
            telemetry.addData("time: ", clock.milliseconds());
            telemetry.update();
        }
    }

    public void drive(double turn, double drive_x, double drive_y, double time) {
        counter = getRuntime() * 1000.0;
        while (getRuntime() * 1000.0 < counter + time) {
            drive(turn, drive_x, drive_y);
        }
        stopRobot();
    }

    public void stopRobot(double time) {
        counter = getRuntime() * 1000.0;
        while (getRuntime() * 1000.0 < counter + time) {
            drive(0, 0, 0);
        }
    }

    public void stopRobot() {
        drive(0, 0, 0);
    }

    public void drive(double turn, double drive_x, double drive_y) {
        double leftPower;
        double rightPower;
        double startTime = clock.milliseconds();

        telemetry.addData("CBL R,G,B", "(" + CBL.red() + ", " + CBL.green() + ", " + CBL.blue() + ")");

        if (Math.abs(turn) < .15) {
            turn = 0;
        }

        if (Math.abs(drive_y) > .15) {
            telemetry.addData("Status", "Driving");
            strafe(false);

            leftPower = Range.clip(drive_y + turn, -1.0, 1.0);
            rightPower = Range.clip(drive_y - turn, -1.0, 1.0);

            FL.setPower(leftPower);
            BL.setPower(leftPower);
            FR.setPower(rightPower);
            BR.setPower(rightPower);
        } else if (Math.abs(drive_x) > .15) {
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
        if (CBL.red() > CBL.blue()) {
            return JewelPosition.RED_JEWEL_RIGHT;
        } else if (CBL.red() < CBL.blue()) {
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

    public void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vufParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        vufParameters.vuforiaLicenseKey = "Aeba4Qn/////AAAAGahNOxzreUE8nItPWzsrOlF7uoyrR/qbjue3kUmhxZcfZMSd5MRyEY+3uEoVA+gpQGz5KyP3wEjBxSOAb4+FBYMZ+QblFU4byMG4+aiI+GeeBA+RatQXVzSduRBdniCW4qehTnwS204KTUMXg1ioPvUlbYQmqM5aPMx/2xnYN1b+htNBWV0Bc8Vkyspa0NNgz7PzF1gozlCIc9FgzbzNYoOMhqSG+jhKf47SZQxD6iEAtj5iAkWBvJwFDLr/EfDfPr3BIA6Cpb4xaDc0t4Iz5wJ/p4oLRiEJaLoE/noCcWFjLmPcw9ccwYXThtjC+7u0DsMX+r+1dMikBCZCWWkLzEyjWzy3pOOR3exNRYGZ0vzr";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        vufParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vufParameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();
        telemetry.addData("relic", "activated");
    }

    public RelicRecoveryVuMark getPicto() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            return vuMark;
        }
        clock.reset();
        while (clock.milliseconds() < 2000) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                return vuMark;
            }
        }
        return vuMark;
    }

    double angle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    public void turn(double power, double angle) {
        telemetry.addData("test", "test");
        double startingAngle = angle();
        while (getAngleDiff(startingAngle, angle()) < angle) {
            telemetry.addData("not working", "plz");
            telemetry.addData("angleDiff", getAngleDiff(startingAngle, angle()));
            telemetry.addData("startingAngle", startingAngle);
            if (angle() - getAngleDiff(startingAngle, angle()) < 20.0) {
                drive((power / Math.abs(power)) * .15, 0, 0);
            } else {
                drive(power, 0, 0);
            }
            telemetry.update();
        }
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }

    public void initGyro() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        ultrasonicLeft = hardwareMap.get(AnalogInput.class, "ultrasonicLeft");
        ultrasonicBack = hardwareMap.get(AnalogInput.class, "ultrasonicBack");
    }

    public double getLeftDist() {
        return ultrasonicLeft.getVoltage() * 1000 / 6.4;
    }
    public double getBackDist() {
        return ultrasonicBack.getVoltage() * 1000 / 6.4;
    }

    public void driveUntilLeft(double power, double desiredDist, double tolerance) {
        while (!(getLeftDist() < desiredDist + tolerance && getLeftDist() > desiredDist - tolerance)) {
            telemetry.addData("yeet", getLeftDist());
            if (getLeftDist() < desiredDist - tolerance) {
                drive(0, -power, 0);
            } else {
                drive(0, power, 0);
            }
            telemetry.update();
        }
        drive(0, 0, 0);
    }
    public void driveUntilBack(double power, double desiredDist, double tolerance) {
        while (!(getBackDist() < desiredDist + tolerance && getBackDist() > desiredDist - tolerance)) {
            telemetry.addData("back", getBackDist());
            if (getBackDist() < desiredDist - tolerance) {
                drive(0, 0, power);
            } else {
                drive(0, 0, -power);
            }
            telemetry.update();
        }
        drive(0, 0, 0);
    }
}
