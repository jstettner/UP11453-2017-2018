package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by student on 11/9/17.
 */
@Autonomous(name = "newAutoBlue1")
public class NewAutoBlue1 extends NewAutonomous {
    RelicRecoveryVuMark column = RelicRecoveryVuMark.UNKNOWN;
    public AbstractAutonomous.Alliance getAlliance() {
        return AbstractAutonomous.Alliance.BLUE;
    }
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables.
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        CBL = hardwareMap.get(ColorSensor.class, "CBL");
        CBOT = hardwareMap.get(ColorSensor.class, "CBOT");
        JS = hardwareMap.get(Servo.class, "JS");
        lift = hardwareMap.get(DcMotor.class, "lift");
        SR = hardwareMap.get(Servo.class, "SR");
        SL = hardwareMap.get(Servo.class, "SL");

        FR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        FL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        lift.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        initGyro();
        initVuforia();
        //  initVuforia();
//        new Thread(new Runnable() {
//            @Override
//            public void run() {
//                imu.initialize(parameters);
//                initialized = true;
//            }
//        }).start();
        startingAngle = imu.getAngularOrientation().firstAngle; //grabbers facing away from wall
        telemetry.addData("start", startingAngle);
        telemetry.update();
        strafe(false);
        JS.setPosition(JEWEL_SERVO_UP);
        waitForStart();

        closeGrabber();
        delay(1000);
        lift.setPower(.4);
        delay(800);
        lift.setPower(0);
        column = getPicto();
        telemetry.addData("column", column);
        telemetry.update();
//        pushJewel();
        delay(500);
        drive(0,-.3,0,1400);
        delay(500);
        drive(0,0,-.3,250);
//        driveUntilBack(.25, 7, .6); //center
        delay(500);
        driveUntilColorRed(-.3);
        delay(500);
        drive(0,0,.3,250);
        delay(500);

        // these should become until left
        if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
//            drive(0,0.3,0,325);
            drive(0,0.5,0,700);
        } else if (column == RelicRecoveryVuMark.LEFT) {
            drive(0,0.5,0,700);
        } else if (column == RelicRecoveryVuMark.RIGHT) {
            drive(0,0.3,0,100);
        }

        delay(500);
        turn(-.3, 145);
        delay(500);

        lift.setPower(-.4);
        delay(600);
        lift.setPower(0);
        delay(500);
        openGrabberFlat();
        delay(1000);
        drive(0, 0, .3, 1800);
        delay(500);
        drive(0, 0, -.3, 200);
    }
/*
    public void run(int state) {
        if (state == 0) {
            closeGrabber();
            delay(500);
            lift.setPower(.4);
            delay(500);
            lift.setPower(0);
        } else if (state == 1) {
            column = getPicto();
            telemetry.addData("column", column);
            telemetry.update();
            delay(500);
        } else if (state == 2) {
            pushJewel();
        } else if (state == 3) {
            if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
                drive(0, -.3, 0, 1500);
            } else if (column == RelicRecoveryVuMark.LEFT) {
                drive(0, -.3, 0, 1400);
            } else if (column == RelicRecoveryVuMark.RIGHT) {
                drive(0, -.3, 0, 1700);
            }
        } else if (state == 4) {
            if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
                turn(-.2, 154);
            } else if (column == RelicRecoveryVuMark.LEFT) {
                turn(-.2, 175);
            } else if (column == RelicRecoveryVuMark.RIGHT) {
                turn(-.2, 147);
            }
        } else if (state == 5) {
            lift.setPower(-.4);
            delay(600);
            lift.setPower(0);
            delay(500);
            openGrabberFlat();
            delay(800);
        } else if (state == 6) {
            drive(0, 0, .3, 1800);
        } else {
            delay(1000);
            drive(0, 0, -.3, 200);
        }
    }
    */
}
