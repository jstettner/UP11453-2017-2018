package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by student on 11/9/17.
 */
@Autonomous(name = "DankRedAuto")

@Disabled
public class autoBlue extends NewAutonomous {
    RelicRecoveryVuMark column = RelicRecoveryVuMark.UNKNOWN;
    public Alliance getAlliance() {
        return Alliance.RED;
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
        JS = hardwareMap.get(Servo.class, "JS");
        lift = hardwareMap.get(DcMotor.class, "lift");
        SR = hardwareMap.get(Servo.class, "SR");
        SL = hardwareMap.get(Servo.class, "SL");
        FR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        FL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        lift.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        telemetry.addData("start", startingAngle);
        telemetry.update();
        strafe(false);
        JS.setPosition(JEWEL_SERVO_UP);
        waitForStart();

           /* closeGrabber();
            delay(1000);
            lift.setPower(.4);
            delay(800);
            lift.setPower(0);
            pushJewel();
            drive(0, -.38, 0, 750);
            drive(-.2, 0, 0, 1700);
            drive(0, 0, .3, 1800);
            delay(500);
            openGrabber();
            delay(500);
            drive(0, 0, -.3, 200);*/
        while (opModeIsActive()) {
            //    getGlyph();
            pushJewel();
            break;
        }
    }

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
                turn(-.2, 163);
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
}
