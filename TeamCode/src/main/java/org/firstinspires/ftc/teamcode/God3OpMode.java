package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

/**
 * Created by Raffa on 10/19/17.
 */

abstract class God3OpMode extends OpMode {

    /**
     * Right servo closed position
     */
    static final double RIGHT_SERVO_OPEN = 0.36;
    /**
     * Left servo closed position
     */
    static final double LEFT_SERVO_OPEN = 0.36;

    /**
     * Right servo open position[]p
     */
    static final double RIGHT_SERVO_CLOSED = 0.60;

    /**
     * Left servo open position
     */
    static final double LEFT_SERVO_CLOSED = 0.57;

    static final double RIGHT_SERVO_AJAR = 0.54;

    /**
     * Left servo open position
     */
    static final double LEFT_SERVO_AJAR = 0.53;

    static final double RIGHT_SERVO_FLAT = 0.32;

    /**
     * Left servo open position
     */
    static final double LEFT_SERVO_FLAT = 0.32;
    static final double SHORT_DRIVE_TIME = 100;
    static final double SHORT_DRIVE_POWER = .5;
    static final double JEWEL_SERVO_DOWN = .6;
    static final double JEWEL_SERVO_UP = .2;
    /**
     * Behaviour when the motors are stopped
     */
    static final ZeroPowerBehavior ZERO_POWER_BEHAVIOR = ZeroPowerBehavior.BRAKE;
}
