package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by student on 10/13/17.
 */

@TeleOp(name = "Yeet")
public class YeetGang extends OpMode {


    private DcMotor lift = null;

    @Override
    public void init() {

        lift = hardwareMap.get(DcMotor.class,"jesus");


    }

    @Override
    public void loop() {

        double rt = gamepad1.right_trigger;
        double lt = gamepad1.left_trigger;

        if(lt > .3 && rt > .3){
            lift.setPower(0);

        }else {
            if (rt < .7) {
                lift.setPower(0);
            } else {
                lift.setPower(rt);

            }

            if (lt < .7) {
                lift.setPower(0);
            } else {
                lift.setPower(-lt);

            }
        }




    }
}