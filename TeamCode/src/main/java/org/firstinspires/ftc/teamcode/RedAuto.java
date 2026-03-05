package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous; /** Alliance Wrapper Classes */
@Autonomous(name = "RED High Speed Auto", group = "Autonomous")
public class RedAuto extends AutonomousRoutine {
    @Override protected Alliance getAlliance() { return Alliance.RED; }
}
