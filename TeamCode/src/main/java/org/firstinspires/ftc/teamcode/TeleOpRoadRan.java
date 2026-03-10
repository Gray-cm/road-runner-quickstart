package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.messages.PoseMessage;

@TeleOp(name = "TeleOp RoadRan", group = "RoadRan")
public class TeleOpRoadRan extends OpMode {
    private Intake intake;
    private Flywheel flywheel;
    private Motor fl, fr, bl, br;
    private Servo runner;
    private com.arcrobotics.ftclib.drivebase.MecanumDrive drive;
    // Road Runner MecanumDrive instance for localization
    private org.firstinspires.ftc.teamcode.MecanumDrive rrDrive;

    // State tracking for selection and toggles
    private String allianceStatus = "NOT SELECTED";

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        
        fl = new Motor(hardwareMap, "forwardleft_motor");
        fr = new Motor(hardwareMap, "forwardright_motor");
        bl = new Motor(hardwareMap, "backleft_motor");
        br = new Motor(hardwareMap, "backright_motor");
        runner = hardwareMap.get(Servo.class, "runner");

        drive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(fl, fr, bl, br);
        
        // Initialize Road Runner's MecanumDrive
        rrDrive = new org.firstinspires.ftc.teamcode.MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void init_loop() {
        // Alliance Selection
        if (gamepad1.a) {
            flywheel.setTargetTagId(20); // Blue Goal
            allianceStatus = "BLUE (20)";
        } else if (gamepad1.b) {
            flywheel.setTargetTagId(24); // Red Goal
            allianceStatus = "RED (24)";
        }

        // Try to detect pattern during init
        intake.updatePattern();

        telemetry.addData("Selection", allianceStatus);
        telemetry.addData("Detected Pattern", intake.getDetectedPattern());
        telemetry.addData("Status", "A: Blue, B: Red | Ready to Start");
        telemetry.update();
    }

    @Override
    public void loop() {
        intake.updatePattern();
        // --- DRIVETRAIN ---
        if (drive != null) {
            drive.driveRobotCentric(
                    gamepad1.right_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x);
        }

        // --- FLYWHEEL & TURRET ---
        flywheel.spinning(); // Auto-aim turret
        flywheel.update(gamepad1.right_trigger); // Power flywheel

        // --- INTAKE & SPINDEXER ---
        intake.runIntakeAndBoot(gamepad1.left_trigger, gamepad1.right_bumper);
        if (gamepad1.left_bumper) {
            runner.setPosition(1);
        } else {
            runner.setPosition(0);
        }

        // --- LOCALIZATION & DASHBOARD ---
        // Update Road Runner Pose Estimate
        rrDrive.updatePoseEstimate();
        Pose2d pose = rrDrive.localizer.getPose();

        // Send Pose to Dashboard Field Overlay
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        // --- LOGGING (Messages) ---
        // This logs the current pose to the Flight Recorder (visible in RR Log Viewer)
        FlightRecorder.write("TELEOP_POSE", new PoseMessage(pose));

        // --- TELEMETRY ---
        telemetry.addData("Alliance", allianceStatus);
        telemetry.addData("Pattern", intake.getDetectedPattern());
        telemetry.addData("Flywheel Velocity", (int)flywheel.getVelocity());
        telemetry.addData("X", pose.position.x);
        telemetry.addData("Y", pose.position.y);
        telemetry.addData("Heading", Math.toDegrees(pose.heading.toDouble()));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (drive != null) {
            fl.stopMotor();
            fr.stopMotor();
            bl.stopMotor();
            br.stopMotor();
        }
    }
}
