package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Old Robot - BLUE Auto", group = "OldRobot")
public class OldBlueAuto extends OldRobotAutonomous {
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}
