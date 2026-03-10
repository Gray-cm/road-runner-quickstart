package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.messages.ThreeDeadWheelInputsMessage;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    public static class Params {
        public double par0Y = 6.75; // y position of the first parallel encoder (in inches)
        public double par1Y = -6.75; // y position of the second parallel encoder (in inches)
        public double perpX = -6.0; // x position of the perpendicular encoder (in inches)
    }

    public static Params PARAMS = new Params();

    public final Encoder par0, par1, perp;

    public final double inPerTick;
    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
    private boolean initialized;
    private Pose2d pose;

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "spinner")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "boot")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "Shoot2")));

        // TODO: reverse encoder directions if needed
        //   par0.setDirection(DcMotorSimple.Direction.REVERSE);

        this.inPerTick = inPerTick;

        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);

        pose = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public PoseVelocity2d update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS", new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));

        if (!initialized) {
            initialized = true;

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;

            return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        }

        double par0PosDelta = (par0PosVel.position - lastPar0Pos) * inPerTick;
        double par1PosDelta = (par1PosVel.position - lastPar1Pos) * inPerTick;
        double perpPosDelta = (perpPosVel.position - lastPerpPos) * inPerTick;

        double headingDelta = (par0PosDelta - par1PosDelta) / (PARAMS.par0Y - PARAMS.par1Y);
        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.par0Y * par1PosDelta - PARAMS.par1Y * par0PosDelta) / (PARAMS.par0Y - PARAMS.par1Y),
                                (PARAMS.par0Y * par1PosVel.velocity - PARAMS.par1Y * par0PosVel.velocity) / (PARAMS.par0Y - PARAMS.par1Y),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                (PARAMS.perpX / (PARAMS.par0Y - PARAMS.par1Y) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.perpX / (PARAMS.par0Y - PARAMS.par1Y) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        (par0PosVel.velocity - par1PosVel.velocity) * inPerTick / (PARAMS.par0Y - PARAMS.par1Y),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        pose = pose.plus(twist.value());
        return twist.velocity().value();
    }
}
