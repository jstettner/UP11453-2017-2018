package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by student on 11/9/17.
 */
@Autonomous(name = "newAutoRedActive1")
public class NewAutoRedActive1 extends NewAutonomous {
    RelicRecoveryVuMark column = RelicRecoveryVuMark.UNKNOWN;
    double beforeAngle = 0;
    public Alliance getAlliance() {
        return Alliance.RED;
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

        startingAngle = imu.getAngularOrientation().firstAngle; //grabbers facing away from wall
        telemetry.addData("start", startingAngle);
        telemetry.update();
        strafe(false);
        JS.setPosition(JEWEL_SERVO_UP);
        liftServo.setPosition(LIFT_FLIPDOWN);
        waitForStart();
        column = getPicto();
        intake(.81);
        lift(.7, 500);
        stopIntake();
        pushJewel();
        delay(500);
        drive(0,.3,0,1400);
        delay(500);
        drive(0,0,-.3,600);
        delay(500);
        driveUntilColorRed(.3);
        delay(500);
        drive(0,0,.3,400);
        delay(500);

        // these should become until left
        if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
            drive(0,-0.3,0,520);
            delay(500);
            turn(.3, 130);
            delay(500);
        } else if (column == RelicRecoveryVuMark.LEFT) {
            drive(0,0.3,0,400);
            delay(500);
            turn(.3, 130);
            delay(500);
        } else if (column == RelicRecoveryVuMark.RIGHT) {
            drive(0,0.3,0,950);
            delay(500);
            turn(-.3, 140);
            delay(500);
        }
        drive(0, 0, .3, 1800);
        lift(-.7, 500);
        outtake(.81);
        delay(500);
        drive(0, 0, -.3, 200);
    }

//    public void run(int state) {
//        if (state == 0) {
//            closeGrabber();
//            delay(1000);
//            lift.setPower(.4);
//            delay(800);
//            lift.setPower(0);
//        } else if (state == 1) {
//        //    column = getPicto();
//            telemetry.addData("column", column);
//            telemetry.update();
//        } else if (state == 2) {
//            pushJewel();
//        } else if (state == 3) {
//            delay(500);
//            if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
//                drive(0, .3, 0, 1500);
//            } else if (column == RelicRecoveryVuMark.LEFT) {
//                drive(0, .3, 0, 1500);
//            } else if (column == RelicRecoveryVuMark.RIGHT) {
//                drive(0, .3, 0, 1500);
//            }
//        } else if (state == 4) {
//            if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
//                turn(.2, 155);
//            } else if (column == RelicRecoveryVuMark.LEFT) {
//                turn(.2, 175);
//            } else if (column == RelicRecoveryVuMark.RIGHT) {
//                turn(.2, 152);
//            }
//        } else if (state == 5) {
//            lift.setPower(-.4);
//            delay(600);
//            lift.setPower(0);
//            delay(500);
//            openGrabberFlat();
//            delay(1000);
//        } else if (state == 6) {
//            drive(0, 0, .3, 1800);
//        } else {
//            delay(1000);
//            drive(0, 0, -.3, 200);
//        }
//    }
}
