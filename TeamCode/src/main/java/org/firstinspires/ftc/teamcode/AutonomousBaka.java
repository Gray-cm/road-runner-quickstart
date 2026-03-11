package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Autonomous Baka", group = "Autonomous")
public class AutonomousBaka extends LinearOpMode {
    Flywheel myFly;
    Drivetrain myDrive;
    Intake myIntake;
    int targetId = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        myDrive = new Drivetrain(hardwareMap);
        myFly = new Flywheel(hardwareMap);
        myIntake = new Intake(hardwareMap);

        // Pre-start selection
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a) {
                targetId = 20; // Blue
            } else if (gamepad1.b) {
                targetId = 24; // Red
            }
            myFly.setTargetTagId(targetId);
            
            myIntake.updatePattern();

            telemetry.addData("Target ID", targetId == -1 ? "NONE (Press A or B)" : targetId);
            telemetry.addData("Detected Pattern", myIntake.getDetectedPattern());
            telemetry.addData("Voltage", "%.2fV", myIntake.getVoltage());
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        // --- SEQUENCE ---
        
        // 1. Drive forward slightly
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 1.5) {
            myDrive.drive(5000, 0, 0);
        }
        myDrive.stopMotors();

        // 2. Align and Shoot using Limelight
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 4.0) {
            myFly.spinning(); // Align turret
            myFly.autoShoot(); // Spin up flywheel to correct speed based on distance

            // If aligned well enough, we could trigger a kicker if it were automated
            // For now, let's just run it for a bit to ensure it hits target speed
        }
        
        // 3. Kick the ball (using intake's boot)
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 1.0) {
            myFly.spinning();
            myFly.autoShoot();
            myIntake.runIntakeAndBoot(0, true); // Kick
        }
        
        myIntake.runIntakeAndBoot(0, false);
        myFly.update(0); // Stop flywheel

        // 4. Drive back
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 1.0) {
            myDrive.drive(-0.4, 0, 0);
        }
        myDrive.stopMotors();
        myFly.stopp();
        myIntake.stopp();
        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }
}
