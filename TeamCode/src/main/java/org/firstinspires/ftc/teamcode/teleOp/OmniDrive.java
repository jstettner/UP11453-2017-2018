package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

@TeleOp(name="OmniDrive")

public class OmniDrive extends OpMode
{
    /**
     * Minimum acceptable motor power
     */
    public static final float MIN_POWER = .3f;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    /**
     * Front right motor.
     */
    private DcMotor FRDrive = null;

    /**
     * Back right motor.
     */
    private DcMotor BRDrive = null;

    /**
     * Front left motor.
     */
    private DcMotor FLDrive = null;

    /**
     * Back left motor.
     */
    private DcMotor BLDrive = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FRDrive = hardwareMap.get(DcMotor.class, "fr_drive");
        BRDrive = hardwareMap.get(DcMotor.class, "br_drive");
        FLDrive = hardwareMap.get(DcMotor.class, "fl_drive");
        BLDrive = hardwareMap.get(DcMotor.class, "bl_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Get the x and y of the left stick of the gamepad.
        float x = -gamepad1.left_stick_x;
        float y = gamepad1.left_stick_y;
        // Calculate the power of the motors.
        float powerFR = correctPower(y - x);
        float powerFL = correctPower(y + x);
        float powerBR = correctPower(y + x);
        float powerBL = correctPower(y - x);

        // Set the power of the motors.
        FRDrive.setPower(powerFR);
        FLDrive.setPower(powerFL);
        BRDrive.setPower(powerBR);
        BLDrive.setPower(powerBL);

        // Log the motor powers.
        telemetry.addData("Front right power", powerFR);
        telemetry.addData("Front left power", powerFL);
        telemetry.addData("Back right power", powerBR);
        telemetry.addData("Back left power", powerBL);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /**
     * Make sure the motor power is not too low (below {@link OmniDrive#MIN_POWER MIN_POWER}).
     * @param power The uncorrected power.
     * @return The corrected power (either P = 0 or P >= .7).
     */
    public float correctPower(float power) {
        if (abs(power) < MIN_POWER) {
            return 0f;
        } else if (abs(power) >= MIN_POWER && abs(power) <= 1f) {
            return power;
        } else {
            return 0f;
        }
    }

}
