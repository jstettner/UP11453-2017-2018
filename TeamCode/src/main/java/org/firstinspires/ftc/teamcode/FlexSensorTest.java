package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "JackSucks")
@Disabled
public class FlexSensorTest extends God3OpMode {
    // Declare OpMode members.
    ElapsedTime clock = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        // Tell the driver that the Op Mode has started
        telemetry.addData("Status", "Initialized");
        AnalogInput flex = hardwareMap.get(AnalogInput.class, "flex");

        // Wait for the start button to be pressed on the phone.
        clock.reset();
        waitForStart();

        // Loop until the op mode is stopped.
     //   while (!isStopRequested() && opModeIsActive()) {
        while (opModeIsActive()) {
            telemetry.addData("runtime", getRuntime() * 1000.0);
            telemetry.addData("runtime", clock.milliseconds());
            telemetry.update();
        }

      //  }

    }
    public void delay(int time) {
        telemetry.addData("delay", "started delay");
        clock.reset();
        while (time > clock.milliseconds()) {
            telemetry.addData("time: ", clock.milliseconds());
            telemetry.update();
        }
    }

    /*

     * Switch t
}
 */
}