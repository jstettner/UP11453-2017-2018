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
    static final double RIGHT_SERVO_CLOSED = 0.3;
    /**
     * Left servo closed position
     */
    static final double LEFT_SERVO_CLOSED = 0.3;

    /**
     * Right servo open position
     */
    static final double RIGHT_SERVO_OPEN = 0.75;

    /**
     * Left servo open position
     */
    static final double LEFT_SERVO_OPEN = 0.8;

    /**
     * Behaviour when the motors are stopped
     */
    static final ZeroPowerBehavior ZERO_POWER_BEHAVIOR = ZeroPowerBehavior.BRAKE;
}
