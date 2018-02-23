package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Sensor Test")
//@Disabled
public class SensorTest extends God3OpMode {
    // Declare OpMode members.
    ElapsedTime clock = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        // Tell the driver that the Op Mode has started
        telemetry.addData("Status", "Initialized");

        // Wait for the start button to be pressed on the phone.
        clock.reset();
        AnalogInput ultrasonic_left = hardwareMap.get(AnalogInput.class, "ultrasonicLeft");
        AnalogInput ultrasonic_right = hardwareMap.get(AnalogInput.class, "ultrasonicRight");
        AnalogInput ultrasonic_back = hardwareMap.get(AnalogInput.class, "ultrasonicBack");
        ColorSensor CBOT = hardwareMap.get(ColorSensor.class, "CBOT");
        waitForStart();
          while (opModeIsActive() && !isStopRequested()) {
              // We multiply the voltage by 1000 since the given conversion rate is in millivolts.
              telemetry.addData("distance left", ultrasonic_left.getVoltage() * 1000 / 6.4);
              telemetry.addData("distance right", ultrasonic_right.getVoltage() * 1000 / 6.4);
              telemetry.addData("distance back", ultrasonic_back.getVoltage() * 1000 / 6.4);
              telemetry.addData("color sensor bottom", "red: " + CBOT.red()+", blue: " + CBOT.blue());
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