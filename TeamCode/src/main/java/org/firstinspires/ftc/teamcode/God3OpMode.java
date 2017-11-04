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
    static final double RIGHT_SERVO_OPEN = 0.4;
    /**
     * Left servo closed position
     */
    static final double LEFT_SERVO_OPEN = 0.4;

    /**
     * Right servo open position[]p
     */
    static final double RIGHT_SERVO_CLOSED = 0.6;

    /**
     * Left servo open position
     */
    static final double LEFT_SERVO_CLOSED = 0.57;

    static final double RIGHT_SERVO_FLAT = 0.25;

    /**
     * Left servo open position
     */
    static final double LEFT_SERVO_FLAT = 0.25;

    /**
     * Behaviour when the motors are stopped
     */
    static final ZeroPowerBehavior ZERO_POWER_BEHAVIOR = ZeroPowerBehavior.BRAKE;
}
