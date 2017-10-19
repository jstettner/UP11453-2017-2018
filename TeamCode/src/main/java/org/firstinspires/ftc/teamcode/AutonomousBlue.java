package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "autoBlue")

public class AutonomousBlue extends AbstractAutonomous {

    @Override
    public Alliance getAlliance() {
        return Alliance.BLUE;
    }
}
