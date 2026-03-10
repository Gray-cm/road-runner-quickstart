package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Old Robot TeleOp", group = "OldRobot")
public class OldTeleOp extends LinearOpMode {
    private OldRobotIntake intake;
    private OldRobotFlywheel flywheel;
    private OldRobotMecanumDrive drive;
    private String allianceStatus = "NOT SELECTED";

    @Override
    public void runOpMode() {
        // 1. INITIALIZE SUBSYSTEMS
        intake = new OldRobotIntake(hardwareMap);
        flywheel = new OldRobotFlywheel(hardwareMap);
        drive = new OldRobotMecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // 2. ALLIANCE SELECTION (In Init)
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) {
                flywheel.setTargetTagId(20); // Blue Goal
                allianceStatus = "BLUE (20)";
            } else if (gamepad1.b) {
                flywheel.setTargetTagId(24); // Red Goal
                allianceStatus = "RED (24)";
            }
            
            telemetry.addData("Selection", allianceStatus);
            telemetry.addData("Status", "X: Blue, B: Red | Ready to Start");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            // Get current target data from Flywheel's internal camera
            AprilTagDetection target = flywheel.getTargetDetection();

            // 3. DRIVING & AUTO-SNAP (Button A)
            if (gamepad1.a && target != null && target.ftcPose != null) {
                drive.autoAlign(target.ftcPose.bearing);
            } else {
                drive.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            // 4. INTAKE (Left Trigger = In, Left Bumper = Out)
            if (gamepad1.left_trigger > 0.1) {
                intake.spinning(0.8, false);
            } else if (gamepad1.left_bumper) {
                intake.spinning(-0.8, false);
            } else {
                intake.spinning(0, false);
            }

            // 5. SHOOTER (Right Trigger = Power, Button X = Auto-Velocity)
            if (gamepad1.x) {
                double autoVel = flywheel.getAutoVelocity();
                flywheel.update(autoVel);
                telemetry.addData("Shooter", "AUTO: " + (int)autoVel);
            } else if (gamepad1.right_trigger > 0.1) {
                flywheel.update(gamepad1.right_trigger); // TeleOp Power mode
                telemetry.addData("Shooter", "MANUAL: " + (int)(gamepad1.right_trigger * 100) + "%");
            } else {
                flywheel.update(0);
                telemetry.addData("Shooter", "OFF");
            }

            // 6. BOOT/KICK (Right Bumper)
            if (gamepad1.right_bumper) {
                intake.spinning(0, true);
            }

            // TELEMETRY
            telemetry.addData("Target Status", target != null ? "LOCKED" : "NO TAG");
            if (target != null && target.ftcPose != null) {
                telemetry.addData("Distance", "%.2f in", target.ftcPose.range);
                telemetry.addData("Bearing", "%.2f deg", target.ftcPose.bearing);
            }
            telemetry.update();
        }

        // Cleanup
        intake.close();
        flywheel.close();
    }
}
