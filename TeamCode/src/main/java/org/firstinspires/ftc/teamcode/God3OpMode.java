package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

/**
 * Created by Raffa on 10/19/17.
 */

abstract class God3OpMode extends LinearOpMode {
    /**
     * The power with which to turn when knocking off the jewel.
     */
    private static final double JEWEL_TURN_TIME = 125;

    /**
     * Clock to time operations
     */
    private ElapsedTime clock = new ElapsedTime();

    /**
     * Right color sensor
     */
     ColorSensor CBR;

    /**
     * Left color sensor
     */
     ColorSensor CBL;

    /**
     * Front-right Servo
     */
     DcMotor FR = null;

    /**
     * Front-left Servo
     */
     DcMotor FL = null;

    /**
     * Back-right Servo
     */
     DcMotor BR = null;

    /**
     * Back-left Servo
     */
     DcMotor BL = null;

    /**
     * Jewel Servo
     */
     Servo JS = null;

    /**
     * Left grabber servo
     */
     Servo SL = null;

    /**
     * Right grabber servo
     */
     Servo SR = null;

     DcMotor lift = null;
    /**
     * Right servo closed position
     */
    static final double RIGHT_SERVO_OPEN = 1 - 0.36;
    /**
     * Left servo closed position
     */
    static final double LEFT_SERVO_OPEN = 0.36;

    /**
     * Right servo open position[]p
     */
    static final double RIGHT_SERVO_CLOSED = 1 - 0.73;


    /**
     * Left servo open position
     */
    static final double LEFT_SERVO_CLOSED = 0.60;

    static final double RIGHT_SERVO_AJAR = 1 - 0.62;

    static final double RELIC_UNGRIPPED = 0.60;

    static final double RELIC_GRIPPED = 0.15;

    static final double RELIC_PICKUP = .28;

    static final double RELIC_DROP = .4;


    /**
     * Left servo open position
     */
    static final double LEFT_SERVO_AJAR = 0.53;

    static final double RIGHT_SERVO_FLAT = 1 - 0.32;

    /**
     * Left servo open position
     */
    static final double LEFT_SERVO_FLAT = 0.32;
    static final double SHORT_DRIVE_TIME = 100;
    static final double SHORT_DRIVE_POWER = .5;
    static final double JEWEL_SERVO_DOWN = .66;
    static final double JEWEL_SERVO_UP = .2;
    /**
     * Behaviour when the motors are stopped
     */
    static final ZeroPowerBehavior ZERO_POWER_BEHAVIOR = ZeroPowerBehavior.BRAKE;
    public void strafe(boolean strafe) {
        FR.setDirection(strafe ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        FL.setDirection(strafe ? DcMotor.Direction.FORWARD : DcMotor.Direction.FORWARD);
        BR.setDirection(strafe ? DcMotor.Direction.REVERSE : DcMotor.Direction.REVERSE);
        BL.setDirection(strafe ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
    }
    public double getAngleDiff(double angle1, double angle2) {
        if(Math.abs(angle1 - angle2) < 180.0)
            return Math.abs(angle1-angle2);
        else if(angle1 > angle2)
        {
            angle1 -= 360;
            return Math.abs(angle2-angle1);
        }
        else
        {
            angle2 -= 360;
            return Math.abs(angle1-angle2);
        }
    }
}
