package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * New automous Op Mode
 */

public abstract class NewAutonomous extends God3OpMode {

    /**
     * The original angle
     */
    private double startingAngle = 0;

    /**
     * The REV imu
     */
    private BNO055IMU imu = null;

    @Override
    public void init() {

        imu = hardwareMap.get(BNO055IMU.class, "imu");

    }

    volatile boolean started = false;

    @Override
    public void start() {
        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

//        new Thread(new Runnable() {
//            @Override
//            public void run() {
//                imu.initialize(parameters);
//                initialized = true;
//            }
//        }).start();
        imu.initialize(parameters);
        startingAngle = imu.getAngularOrientation().firstAngle;
        telemetry.addData("start", startingAngle);
        telemetry.update();
        started = true;
    }

//    private volatile boolean initialized = false;

    private int countThing = 0;

    @Override
    public void loop() {

        if (!started) {
            return;
        }

//        if (countThing < 30) {
//            countThing++;
//            return;
//        } else if (countThing == 30) {
//            countThing++;
//            startingAngle = imu.getAngularOrientation().firstAngle;
//            return;
//        }

        telemetry.addData("First", imu.getAngularOrientation().firstAngle);
        telemetry.addData("Second", imu.getAngularOrientation().secondAngle);
        telemetry.addData("Third", imu.getAngularOrientation().thirdAngle);
        telemetry.addData("Orientation",angle());
    }

    private double angle() {
        double pos = (imu.getAngularOrientation().firstAngle - startingAngle + 360) % 360;
        if (pos <= 180) {
            return pos;
        } else {
            return pos - 360;
        }
    }
}
