/*package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroPathing extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain line2;
        public PathChain line3;
        public PathChain line4;
        public PathChain line5;
        public PathChain line6;
        public PathChain line7;
        public PathChain line8;
        public PathChain line9;
        public PathChain line10;
        public PathChain line11;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(27.000, 131.500),
                                    new Pose(71.260, 84.277)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-38.5), Math.toRadians(180))
                    .build();

            line2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(71.260, 84.277),
                                    new Pose(12.208, 84.069)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            line3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(12.208, 84.069),
                                    new Pose(71.306, 84.347)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            line4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(71.306, 84.347),
                                    new Pose(71.584, 60.486)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            line5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(71.584, 60.486),
                                    new Pose(11.653, 59.931)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            line6 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(11.653, 59.931),
                                    new Pose(71.861, 60.486)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            line7 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(71.861, 60.486),
                                    new Pose(71.306, 84.347)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            line8 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(71.306, 84.347),
                                    new Pose(70.751, 36.069)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            line9 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(70.751, 36.069),
                                    new Pose(11.653, 36.069)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            line10 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(11.653, 36.069),
                                    new Pose(70.751, 36.069)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            line11 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(70.751, 36.069),
                                    new Pose(71.306, 84.624)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return 0;
    }
}

*/