package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "autoBlue")
@Disabled
public class AutonomousBlue extends AbstractAutonomous {

    @Override
    public Alliance getAlliance() {
        return Alliance.BLUE;
    }
}
