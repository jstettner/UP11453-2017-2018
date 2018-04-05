package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class LiftTest extends God3OpMode {
    // Declare OpMode members.



    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void runOpMode() throws InterruptedException {
        Servo flipServo = hardwareMap.get(Servo.class, "flipServo");
        boolean read = false;
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.a) {
                flipServo.setPosition(.95);
            } else if (gamepad2.x) {
                flipServo.setPosition(.05);
            } else if (gamepad2.y) {
               flipServo.setPosition(.99);
            } else if (gamepad2.b){
                flipServo.setPosition(0.01);
            }
            if (gamepad2.right_stick_y > .2) {
                flipServo.setPosition(.91);
            }


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
