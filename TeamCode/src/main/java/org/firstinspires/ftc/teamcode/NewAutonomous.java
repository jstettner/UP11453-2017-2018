package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * New autonomous Op Mode
 */

public abstract class NewAutonomous extends AbstractAutonomous {

    /**
     * The original angle
     */
    double startingAngle = 0;

    private ElapsedTime clock = new ElapsedTime();


    public void delay(int time) {
        double startTime = clock.milliseconds();
        while (!isStopRequested() && opModeIsActive() && clock.milliseconds() - startTime < time) {
        }
    }


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
        SR.setPosition(RIGHT_SERVO_FLAT);
        SL.setPosition(LEFT_SERVO_FLAT);
    }

    public void drive(double turn, double drive_x, double drive_y, int time) {
        boolean driving = true;
        while (!isStopRequested() && opModeIsActive() && driving) {
            if (!isStopRequested() && opModeIsActive()) drive(turn, drive_x, drive_y);
            if (!isStopRequested() && opModeIsActive()) delay(time);
            if (!isStopRequested() && opModeIsActive()) drive(0, 0, 0);
            if (!isStopRequested() && opModeIsActive()) driving = false;
        }
    }

    public void drive(double turn, double drive_x, double drive_y) {
        double leftPower;
        double rightPower;

        telemetry.addData("CBL R,G,B", "(" + CBL.red() + ", " + CBL.green() + ", " + CBL.blue() + ")");

        if (Math.abs(turn) < .05) {
            turn = 0;
        }

        if (Math.abs(drive_y) > .05) {
            telemetry.addData("Status", "Driving");
            strafe(false);

            leftPower = Range.clip(drive_y + turn, -1.0, 1.0);
            rightPower = Range.clip(drive_y - turn, -1.0, 1.0);

            FL.setPower(1.1 * leftPower / 1.4);
            BL.setPower(1.1 * leftPower);
            FR.setPower(1.1 * rightPower);
            BR.setPower(1.1 * rightPower / 1.4);
        } else if (Math.abs(drive_x) > .05) {
            telemetry.addData("Status", "Strafing");
            strafe(true);

            leftPower = Range.clip(drive_x + turn, -1.0, 1.0);
            rightPower = Range.clip(drive_x - turn, -1.0, 1.0);
            if (drive_x < 0) {
                FL.setPower(leftPower);
                BL.setPower(rightPower * 1.2);
                FR.setPower(leftPower * 1.2);
                BR.setPower(rightPower);
            } else {
                FL.setPower(leftPower);
                BL.setPower(rightPower);
                FR.setPower(leftPower);
                BR.setPower(rightPower);
            }
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

    double angle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }
}
