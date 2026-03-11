package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@Autonomous(name="!!! 12-BALL MASTER AUTO !!!", group="Production")
public class MasterTwelveBallAuto extends LinearOpMode {


    // Hardware
    private Limelight3A limelight;
    private CRServo turretServo;
    private DcMotorEx m0, m1;
    private VoltageSensor battery;


    // Configuration & Alliance Selection
    private int allianceID = -1; // Blue by default
    private double side = -1.0;  // Blue = -1, Red = 1


    @Override
    public void runOpMode() {
        // 1. Initialize Hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(CRServo.class, "spin");
        m0 = hardwareMap.get(DcMotorEx.class, "Shoot1");
        m1 = hardwareMap.get(DcMotorEx.class, "Shoot2");
        battery = hardwareMap.voltageSensor.iterator().next();


        // 2. Initialize Drivetrain (Uses your 3-pod localizer automatically)
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        // 3. Alliance Selection (X for Blue, B for Red)
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) { allianceID = 20; side = -1.0; }
            if (gamepad1.b) { allianceID = 24; side = 1.0; }


            telemetry.addData("TEAM", allianceID == 20 ? "BLUE" : "RED");
            telemetry.addLine("READY - PRESS PLAY");
            telemetry.update();
        }


        waitForStart();


        // 4. Build the Drive Path
        Action drivePath = drive.actionBuilder(new Pose2d(0, 0, 0))
                // Path to Launching Triangle
                .strafeToLinearHeading(new Vector2d(24, 24 * side), Math.toRadians(45 * side))
                .waitSeconds(0.8) // Time to fire preloads


                // Path to Submersible/Intake
                .strafeToLinearHeading(new Vector2d(12, 48 * side), Math.toRadians(90 * side))
                .waitSeconds(1.0) // Intake time


                // Return to Launch Zone
                .strafeToLinearHeading(new Vector2d(24, 24 * side), Math.toRadians(45 * side))
                .build();


        // 5. Build the Turret Tracking Action
        Action turretAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Stop the action if OpMode ends
                if (!opModeIsActive() || isStopRequested()) return false;


                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    double tx = result.getTx();
                    turretServo.setPower(tx * 0.025); // Proportional lock-on


                    // Battery compensation for consistent shots
                    double comp = 12.0 / battery.getVoltage();
                    m0.setVelocity(2100 * comp);
                    m1.setVelocity(2100 * comp);
                    packet.put("Lock", "ON");
                } else {
                    turretServo.setPower(0);
                    packet.put("Lock", "SEARCHING");
                }
                // Returning true keeps this running simultaneously with drivePath
                return true;
            }
        };


        // 6. Execute Parallel Logic
        // This runs both actions at once. It ends when drivePath finishes.
        Actions.runBlocking(
                new ParallelAction(
                        drivePath,
                        turretAction
                )
        );
    }
}



