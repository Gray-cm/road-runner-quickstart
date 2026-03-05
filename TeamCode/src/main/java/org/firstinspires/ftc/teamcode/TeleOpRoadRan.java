package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp RoadRan", group = "RoadRan")
public class TeleOpRoadRan extends OpMode {
    private Intake intake;
    private Flywheel flywheel;
    private Motor fl, fr, bl, br;
    private com.arcrobotics.ftclib.drivebase.MecanumDrive drive;

    // State tracking for selection and toggles
    private String allianceStatus = "NOT SELECTED";
    private boolean lastX = false;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        
        fl = new Motor(hardwareMap, "forwardleft_motor");
        fr = new Motor(hardwareMap, "forwardright_motor");
        bl = new Motor(hardwareMap, "backleft_motor");
        br = new Motor(hardwareMap, "backright_motor");

        drive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(fl, fr, bl, br);
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
        // Intake spinner: Left Trigger
        // Boot/Kick: Right Bumper
        intake.runIntakeAndBoot(gamepad1.left_trigger, gamepad1.right_bumper);

        // Manual Spindexer Slot Control (D-Pad)
        /*
        if (gamepad1.dpad_up) {
            intake.setSpindexerToSlot(1);
        } else if (gamepad1.dpad_right) {
            intake.setSpindexerToSlot(2);
        } else if (gamepad1.dpad_down) {
            intake.setSpindexerToSlot(3);
        }*/

        // Automatic Spindexer Cycle (using the sorting logic)
        // Pressing 'X' cycles to the next best ball based on the detected pattern
        /*if (gamepad1.x && !lastX) {
             intake.positionNextBall();
        }
        lastX = gamepad1.x;

        // Reset the spindexer index or scan pattern
        if (gamepad1.y) {
            intake.resetScoringIndex();
            intake.updatePattern(); // Try scanning again if it was missed
        }
*/
        // --- TELEMETRY ---
        telemetry.addData("Alliance", allianceStatus);
        telemetry.addData("Pattern", intake.getDetectedPattern());
        telemetry.addData("Flywheel Velocity", (int)flywheel.getVelocity());
        //telemetry.addData("Is it Green?", intake.isGreenPixel());
        telemetry.addData("Drivetrain Power", fl.getCurrentPosition());
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
