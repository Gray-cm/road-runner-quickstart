package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

public class TrapezoidalMovement {
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(10,20);
    TrapezoidProfile.State goal = new TrapezoidProfile.State(5,0);
    TrapezoidProfile.State init = new TrapezoidProfile.State(0,0);
    TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, init);
    private double rkP;
    private double rkI;
    private double rkD;
    ProfiledPIDController controller = new ProfiledPIDController(rkP,rkI,rkD,constraints);

    // to set goal for a motor do MOTOR.set(controller.calculate(encoder.getDistance(), goal);
}
