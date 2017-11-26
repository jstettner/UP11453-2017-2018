package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by student on 11/9/17.
 */
@Autonomous(name = "VuMarkYeet")
public class VuMarkYeet extends AbstractAutonomous {

    public Alliance getAlliance() {
        return Alliance.RED;
    }
    public void runOpMode() {
        CBL = hardwareMap.get(ColorSensor.class, "CBL");
        waitForStart();

        while (opModeIsActive()) {
            getPicto();
        }
    }

    public void run(int state) {
        if (state == 0) {
            closeGrabber();
            delay(1000);
            lift.setPower(.4);
            delay(800);
            lift.setPower(0);
        } else if (state == 1) {
            pushJewel();
        } else if (state == 2) {
            delay(500);
            drive(0, .38, 0, 1200);
        } else if (state == 3) {
            drive(.2, 0, 0, 1450);
        } else if (state == 4) {
        } else if (state == 5) {
            lift.setPower(-.4);
            delay(600);
            lift.setPower(0);
            delay(500);
            openGrabberFlat();
            delay(1000);
        } else if (state == 6){
            drive(0, 0, .3, 1800);
        } else {
            delay(500);
            delay(500);
            drive(0, 0, -.3, 200);
        }
    }
}
