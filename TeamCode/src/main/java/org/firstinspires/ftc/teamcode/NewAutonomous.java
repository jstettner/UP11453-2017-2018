package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

/**
 * New automous Op Mode
 */

public abstract class NewAutonomous extends God3OpMode {

    /**
     * The original angle
     */
    private double startingAngle = 0;
    private ElapsedTime clock = new ElapsedTime();
    /**
     * The REV imu
     */
    private BNO055IMU imu = null;
    private double fullAngle = 0;

    @Override
    public void init() {
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
        imu = hardwareMap.get(BNO055IMU.class, "imu");

    }
    boolean turned = false;
    volatile boolean started = false;

    @Override
    public void start() {
        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

//        new Thread(new Runnable() {
//            @Override
//            public void run() {
//                imu.initialize(parameters);
//                initialized = true;
//            }
//        }).start();
        imu.initialize(parameters);
        startingAngle = imu.getAngularOrientation().firstAngle;
        telemetry.addData("start", startingAngle);
        telemetry.update();
        startingAngle = 0;
        started = true;
    }

//    private volatile boolean initialized = false;

    private int countThing = 0;
    int state = 0;
    @Override
    public void loop() {
        if (startingAngle == 0) {
            startingAngle = angle();
        }
        if (!started) {
            return;
        }
        if (state == 0) {
            if (angle() < startingAngle + 75.0) {
                drive(-.15, 0, 0);
            } else {
                drive(0, 0, 0);
                delay(800);
                startingAngle = angle();
                delay(400);
                state = 1;
            }
        }
        while (state == 1) {
            drive(0, 0, .35);
            delay(2000);
            drive(0, 0, 0);
            startingAngle = angle();
            delay(1000);
            state = 2;
        }
        if (state == 2) {
            if (angle() < startingAngle + 20.0) {
                drive(-.12, 0, 0); 
            } else {
                drive(0, 0, 0);
                delay(800);
                startingAngle = angle();
                delay(400);
                state = 3;
            }
        }
        while (state == 3) {
            drive(0, 0, .35);
            delay(500);
            drive(0, 0, 0);
            state = 4;
        }
        //   turn(45.0);
        telemetry.addData("angle: ", angle());
        telemetry.addData("startingAngle: ", startingAngle);


    }

    public void delay(int time) {
        double startTime = clock.milliseconds();
        while (clock.milliseconds() - startTime < time) {
        }
    }

    public void drive(double turn, double drive_x, double drive_y) {
        double leftPower;
        double rightPower;

        telemetry.addData("CBR R,G,B", "(" + CBR.red() + ", " + CBR.green() + ", " + CBR.blue() + ")");
        telemetry.addData("CBL R,G,B", "(" + CBL.red() + ", " + CBL.green() + ", " + CBL.blue() + ")");

        if (Math.abs(turn) < .05) {
            turn = 0;
        }

        if (Math.abs(drive_y) > .05) {
            telemetry.addData("Status", "Driving");
            strafe(false);

            leftPower = Range.clip(drive_y + turn, -1.0, 1.0);
            rightPower = Range.clip(drive_y - turn, -1.0, 1.0);

            FL.setPower(leftPower);
            BL.setPower(leftPower);
            FR.setPower(rightPower);
            BR.setPower(rightPower);
        } else if (Math.abs(drive_x) > .05) {
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
    }

    public void drive(double turn, double drive_x, double drive_y, double time) {
        double leftPower;
        double rightPower;
        double startTime = clock.milliseconds();

        while (clock.milliseconds() - startTime < time) {
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
        }
        telemetry.update();
    }

    void turn(double degrees) {
        boolean turned = false;
        double startingAngle = angle();
        while (!turned) {
            if (degrees > 0) {
                if (angle() < startingAngle + degrees) {
                    drive(-.15, 0, 0);
                } else {
                    drive(0, 0, 0);
                    turned = true;
                }
            }
        }

    }

    double angle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }
}
