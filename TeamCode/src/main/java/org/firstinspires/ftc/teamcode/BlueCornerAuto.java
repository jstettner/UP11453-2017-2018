package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by student on 11/9/17.
 */
@Autonomous(name = "BlueCornerAuto")
public class BlueCornerAuto extends NewAutonomous {
    RelicRecoveryVuMark column = RelicRecoveryVuMark.UNKNOWN;
    public Alliance getAlliance() {
        return Alliance.BLUE;
    }
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
        liftServo = hardwareMap.get(Servo.class, "liftServo");
        rightBottom = hardwareMap.get(CRServo.class, "rightBottom");
        leftBottom = hardwareMap.get(CRServo.class, "leftBottom");
        rightTop = hardwareMap.get(CRServo.class, "rightTop");
        leftTop = hardwareMap.get(CRServo.class, "leftTop");
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
        column = getPicto();
        intake(.81);
        column = getPicto();
        lift(.7, 400);
        stopIntake();
        pushJewel();
        delay(500);
        drive(0,-.3,0,1800);
        delay(500);
        driveUntilColorRedForward(.3); //should be blue
        delay(500);

        if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
            turn(-.3,80);
        } else if (column == RelicRecoveryVuMark.LEFT) {
            turn(-.3,105);
        } else if (column == RelicRecoveryVuMark.RIGHT) {
            turn(-.3,50);
        }

        delay(500);
        drive(0, 0, .3, 300);
        lift(-.7,400);
        outtake(.81);
        delay(1000);
        delay(500);
        drive(0, 0, .3, 1800);
        delay(500);
        stopIntake();
        drive(0, 0, -.3, 350);
    }

//    public void run(int state) {
//        if (state == 0) {
//            closeGrabber();
//            delay(1000);
//            lift.setPower(.4);
//            delay(800);
//            lift.setPower(0);
//        } else if (state == 1) {
//            column = getPicto();
//        } else  if (state == 10) {
//            pushJewel();
//        }if (state == 2) {
//            delay(500);
//            drive(0, -.2, 0, 2500);
//        } else if (state == 3) {
//            delay(500);
//            turn(-.2, 90);
//            delay(500);
//            } else if (state == 4) {
//                delay(500);
//                if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
//                    drive(0, .3, 0, 2000);
//                    turn(-.2, 13);
//                } else if (column == RelicRecoveryVuMark.LEFT) {
//                    drive(0, .3, 0, 1500);
//                    turn(-.2, 10);
//                } else if (column == RelicRecoveryVuMark.RIGHT) {
//                    drive(0, .3, 0, 2500);
//                    turn(-.2, 10);
//                }
//                delay(500);
//            } else if (state == 5) {
//                lift.setPower(-.4);
//                delay(600);
//                lift.setPower(0);
//                delay(500);
//                openGrabberFlat();
//                delay(1000);
//            } else if (state == 6){
//                drive(0, 0, .3, 1800);
//            } else {
//                delay(500);
//                delay(500);
//                drive(0, 0, -.3, 200);
//            }
//        }
    }
