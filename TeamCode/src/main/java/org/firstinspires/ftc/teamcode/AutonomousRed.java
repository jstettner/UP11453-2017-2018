package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "autoRed")

public class AutonomousRed extends AbstractAutonomous {

    @Override
    public Alliance getAlliance() {
        return Alliance.RED;
    }
}