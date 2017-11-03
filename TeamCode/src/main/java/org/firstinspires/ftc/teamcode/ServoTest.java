package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Servo tester Op Mode
 *
 * Created by Raffa on 10/27/17.
 */

@TeleOp(name = "ServoTest")
public class ServoTest extends God3OpMode {

    /**
     * The starting position of the servos
     */
    private static final double SERVO_START_POSITION = 0;

    /**
     * How much to change the servo position when a button is pressed
     */
    private static final double INCREMENT = .01;

    /**
     * The left servo
     */
    private Servo servoLeft = null;

    /**
     * The right servo
     */
    private Servo servoRight = null;

    /**
     * The position of the left servo
     */
    private double servoLeftPosition = SERVO_START_POSITION;

    /**
     * The position of the right servo
     */
    private double servoRightPosition = SERVO_START_POSITION;

    private boolean aPressed;
    private boolean bPressed;
    private boolean xPressed;
    private boolean yPressed;

    @Override
    public void init() {
        // Get the servos from the hardware map
        servoRight = hardwareMap.get(Servo.class, "SR");
        servoLeft = hardwareMap.get(Servo.class, "SL");

        setServoLeftPosition(SERVO_START_POSITION);
        setServoRightPosition(SERVO_START_POSITION);

    }

    @Override
    public void loop() {

        // Get the current gamepad button values
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        // Move the servos if they are newly pressed
        if (a) {
            if (!aPressed) {
                moveServoLeft(INCREMENT);
            }
        } else if (b) {
            if (!bPressed) {
                moveServoLeft(-INCREMENT);
            }
        } else if (x) {
            if (!xPressed) {
                moveServoRight(INCREMENT);
            }
        } else if (y) {
            if (!xPressed) {
                moveServoRight(-INCREMENT);
            }
        }

        // Update the stored gamepad button values
        xPressed = x;
        yPressed = y;
        aPressed = a;
        bPressed = b;
    }

    /**
     * Update the position of the left servo
     * @param position a double between 0.0 and 1.0
     */
    private void setServoLeftPosition(double position) {
        servoLeftPosition = position;
        servoLeft.setPosition(position);

        // Send the new data to telemetry
        telemetry.addData("Left Servo", servoLeftPosition);
    }

    /**
     * Update the position of the right servo
     * @param position a double between 0.0 and 1.0
     */
    private void setServoRightPosition(double position) {
        servoRightPosition = position;
        servoRight.setPosition(position);

        // Send the new data to telemetry
        telemetry.addData("Right Servo", servoRightPosition);
    }

    /**
     * Move the left servo by an amount
     * @param increment how much to move the servo
     */
    private void moveServoLeft(double increment) {
        setServoLeftPosition(servoLeftPosition + increment);
    }

    /**
     * Move the right servo by an amount
     * @param increment how much to move the servo
     */
    private void moveServoRight(double increment) {
        setServoRightPosition(servoRightPosition + increment);
    }
}
