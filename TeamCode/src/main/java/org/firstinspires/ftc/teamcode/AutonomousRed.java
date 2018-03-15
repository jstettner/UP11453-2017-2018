package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "autoRed")
@Disabled
public class AutonomousRed extends AbstractAutonomous {
    @Override
    public Alliance getAlliance() {
        return Alliance.RED;
    }
}
