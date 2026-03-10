package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Old Robot - RED Auto", group = "OldRobot")
public class OldRedAuto extends OldRobotAutonomous {
    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }
}
