package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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

@TeleOp(name = "LiftTest")

public class LiftTest extends God3OpMode {
    // Declare OpMode members.



    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void runOpMode() throws InterruptedException {
        CRServo leftBottom = hardwareMap.get(CRServo.class, "leftBottom");
        CRServo rightBottom = hardwareMap.get(CRServo.class, "rightBottom");
        CRServo leftTop = hardwareMap.get(CRServo.class, "leftTop");
        CRServo rightTop = hardwareMap.get(CRServo.class, "rightTop");
        Servo flipServo = hardwareMap.get(Servo.class, "flipServo");
        boolean read = false;
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.left_trigger > .2) {
                rightTop.setPower(-.81);
                leftTop.setPower(.81);
            } else if (gamepad2.right_trigger > .2) {
                rightBottom.setPower(.81);
                leftBottom.setPower(-.81);
            } else if (Math.abs(gamepad2.left_stick_y) > .2) {
                rightBottom.setPower(-Range.clip(gamepad2.left_stick_y, -.81, .81));
                leftBottom.setPower(Range.clip(gamepad2.left_stick_y, -.81, .81));
                rightTop.setPower(Range.clip(gamepad2.left_stick_y, -.81, .81));
                leftTop.setPower(-Range.clip(gamepad2.left_stick_y, -.81, .81));
            } else {
                // servo test
                rightBottom.setPower(0);
                leftBottom.setPower(0);
                rightTop.setPower(0);
                leftTop.setPower(0);
            }
            if (gamepad2.a) {
                if (!read) {
                    read = true;
                    if (Math.round(flipServo.getPosition() * 100.0) / 100.0 == LIFT_FLIPDOWN) {
                        flipServo.setPosition(LIFT_FLIPUP);
                    } else if (Math.round(flipServo.getPosition() * 100.0) / 100.0 == LIFT_FLIPUP) {
                        flipServo.setPosition(LIFT_FLIPDOWN);
                    } else {
                        flipServo.setPosition(LIFT_FLIPDOWN);
                    }
                }
            } else {
                read = false;
            }

            rightBottom.setPower(-.81);
            leftBottom.setPower(.81);
            telemetry.addData("Right power", rightBottom.getPower());
            telemetry.addData("Left power", leftBottom.getPower());
            telemetry.update();
        }
    }

    /*
     * Scales a value to the appropriate range--used for calculating motor powers/servo positions.
     * For instance, you could use this to map 5 in the range (0,10) to 0.25 in the range (0,0.5)
     */
    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }


}
