package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "red 1", group = "Sensor")
public class QuantumRedAuto1 extends LinearOpMode {
    ElapsedTime clock = new ElapsedTime();

    ColorSensor colorSensor;
    DistanceSensor sensorDistance;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BR = null;
    private DcMotor BL = null;
    private Servo JewelServo = null;
    //    private Servo GlyphServoL = null;
//    private Servo GlyphServoR = null;
//    private DcMotor GlyphWheel1 = null;
//    private DcMotor GlyphWheel2 = null;

    private double moveSpeed = .25;
    private double turnSpeed = .25;

    @Override
    public void runOpMode() {
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        JewelServo = hardwareMap.get(Servo.class, "JS");
//        GlyphWheel1 = hardwareMap.get(DcMotor.class, "GlyphWheel1");
//        GlyphWheel2 = hardwareMap.get(DcMotor.class, "GlyphWheel2");

        JewelServo.setDirection(Servo.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
//        GlyphWheel1.setDirection(DcMotor.Direction.FORWARD);
//        GlyphWheel2.setDirection(DcMotor.Direction.REVERSE);

        //Servo initialization
//        GlyphServoL = hardwareMap.get(Servo.class, "GlyphServo1");
//        GlyphServoL.setDirection(Servo.Direction.REVERSE);
//        GlyphServoR = hardwareMap.get(Servo.class, "GlyphServo2");
//        GlyphServoR.setDirection(Servo.Direction.FORWARD);


        // get a reference to the color sensor.
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorSensor");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // wait for the start button to be pressed.
        waitForStart();

        JewelServo.setPosition(70);
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsvValues);

        telemetry.update();

        //read color
        int red = 0;
        int blue = 0;
        for (int i = 0; i < 40; i++) {
            if (colorSensor.red() > colorSensor.blue()) red++;
            if (colorSensor.red() < colorSensor.blue()) blue++;
            telemetry.update();
        }

        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("RED", red);
        telemetry.addData("BLUE", blue);

        //knock off jewel
        if (red > blue) {
            telemetry.addData("Red Wins!", colorSensor.red());
            telemetry.update();
            moveTime(6,.15);
        } else {
            telemetry.addData("Blue Wins!", colorSensor.red());
            telemetry.update();
            moveTime(5,.15);
        }
        JewelServo.setPosition(0);

        //turn back to initial position
        if(red>blue) {
            moveTime(5,.15);
        } else if(blue>red) {
            moveTime(6,.15);
        }

        //MOVE TO THE CORRECT COLUMN
        moveTime(4, 1.6);

        //turn to face cryptobox
        moveTime(5, 1.614);

        //move forward
        moveTime(1, 1.2);

        //pause
        moveTime(0, 1);

        //release glyph
        moveTime(8, .28);

        //pause
        moveTime(0, 1);

        //move back
        moveTime(2, .25);

        //push back in
        moveTime(1, .3);

        //move back out
        moveTime(2, .25);
    }

    public void moveTime(int dir, double time) {
        double startTime = 0;
        if (dir == 0) {
            startTime = getRuntime();
            while (getRuntime() < startTime + time) {
                driveStop();
            }
        }
        if (dir == 1) {
            startTime = getRuntime();
            while (getRuntime() < startTime + time) {
                moveForward();
            }

        }
        if (dir == 2) {
            startTime = getRuntime();
            while (getRuntime() < startTime + time) {
                moveBackward();
            }
        }
        if (dir == 3) {
            startTime = getRuntime();
            while (getRuntime() < startTime + time) {
                moveLeft();
            }
        }
        if (dir == 4) {
            startTime = getRuntime();
            while (getRuntime() < startTime + time) {
                moveRight();
            }
        }
        if (dir == 5) {
            startTime = getRuntime();
            while (getRuntime() < startTime + time) {
                turnClockwise();
            }
        }
        if (dir == 6) {
            startTime = getRuntime();
            while (getRuntime() < startTime + time) {
                turnCounterClockwise();
            }
        }
        if (dir == 7) {
            startTime = getRuntime();
            while (getRuntime() < startTime + time) {
//                glyphWheels(.6); //pull in
            }
        }
        if (dir == 8) {
            startTime = getRuntime();
            while (getRuntime() < startTime + time) {
//                glyphWheels(-.6); //pull in
            }
        }
    }

    public void glyphWheels(double speed) {
//        GlyphWhee
// l2.setPower(speed);
    }

    public void moveForward() {
        FR.setPower(moveSpeed);
        FL.setPower(-moveSpeed);
        BR.setPower(moveSpeed);
        BL.setPower(-moveSpeed);
    }

    public void moveBackward() {
        FR.setPower(-moveSpeed);
        FL.setPower(moveSpeed);
        BR.setPower(-moveSpeed);
        BL.setPower(moveSpeed);
    }

    public void moveLeft() {
        FR.setPower(moveSpeed);
        FL.setPower(moveSpeed);
        BR.setPower(-moveSpeed);
        BL.setPower(-moveSpeed);
    }

    public void moveRight() {
        FR.setPower(-moveSpeed);
        FL.setPower(-moveSpeed);
        BR.setPower(moveSpeed);
        BL.setPower(moveSpeed);
    }

    public void turnClockwise() {
        FR.setPower(-turnSpeed);
        FL.setPower(-turnSpeed);
        BR.setPower(-turnSpeed);
        BL.setPower(-turnSpeed);
    }

    public void turnCounterClockwise() {
        FR.setPower(turnSpeed);
        FL.setPower(turnSpeed);
        BR.setPower(turnSpeed);
        BL.setPower(turnSpeed);
    }

    public void driveStop() {
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    public void delay(int time) {
        double delayStartTime = clock.milliseconds();
        while (clock.milliseconds() - delayStartTime < time) {
        }
    }

}