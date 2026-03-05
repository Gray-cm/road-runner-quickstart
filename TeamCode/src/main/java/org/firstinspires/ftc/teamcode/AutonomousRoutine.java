package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Abstract base class for high-speed autonomous.
 * Uses mirroring helpers to support both Red and Blue alliances with a single path definition.
 */
public abstract class AutonomousRoutine extends LinearOpMode {
    public enum Alliance { RED, BLUE }
    protected abstract Alliance getAlliance();

    private Intake intake;
    private Flywheel flywheel;
    private MecanumDrive drive;

    // --- ALLIANCE MIRRORING HELPERS ---
    
    public Pose2d mPose(double x, double y, double headingDeg) {
        if (getAlliance() == Alliance.BLUE) {
            return new Pose2d(x, -y, Math.toRadians(-headingDeg));
        }
        return new Pose2d(x, y, Math.toRadians(headingDeg));
    }

    public Vector2d mVec(double x, double y) {
        return (getAlliance() == Alliance.BLUE) ? new Vector2d(x, -y) : new Vector2d(x, y);
    }

    public double mAng(double degrees) {
        return Math.toRadians((getAlliance() == Alliance.BLUE) ? -degrees : degrees);
    }

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        
        Pose2d startPose = mPose(60, 0, 180);
        drive = new MecanumDrive(hardwareMap, startPose);

        // --- BACKGROUND ACTIONS ---
        Action turretTrack = packet -> {
            flywheel.spinning(); // Background turret tracking using tx
            return true; 
        };
        
        Action flywheelAuto = packet -> {
            flywheel.update(flywheel.getAutoVelocity()); // Background velocity adjustment using ty
            return true; 
        };

        // --- SCORING ACTIONS ---
        Action scoreOne = new SequentialAction(
                packet -> { intake.runIntakeAndBoot(0, true); return false; },
                new SleepAction(0.15),
                packet -> { intake.runIntakeAndBoot(0, false); return false; }
        );

        Action scoreThree = new SequentialAction(scoreOne, scoreOne, scoreOne);

        Action intakeOn = packet -> { intake.runIntakeAndBoot(1.0, false); return false; };
        Action intakeOff = packet -> { intake.runIntakeAndBoot(0.0, false); return false; };

        // --- INIT ---
        while (!isStarted() && !isStopRequested()) {
            intake.updatePattern();
            // Crucial Fix: Set the goal target ID based on alliance
            flywheel.setTargetTagId(getAlliance() == Alliance.BLUE ? 20 : 21);
            
            telemetry.addData("Alliance", getAlliance());
            telemetry.addData("Pattern", intake.getDetectedPattern());
            telemetry.addData("Turret Target ID", getAlliance() == Alliance.BLUE ? 20 : 21);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // --- DYNAMIC INTAKE ORDER ---
        Vector2d stack1 = mVec(12, 48);
        Vector2d stack2 = mVec(36, 48);
        Vector2d stack3 = mVec(-11.3, 48);
        
        Vector2d firstStack, secondStack, thirdStack;
        Intake.Pattern p = intake.getDetectedPattern();
        
        // Strategy: Intake Pixels (P) first, Green (G) last or skip
        if (p == Intake.Pattern.GPP) {
            firstStack = stack2; secondStack = stack3; thirdStack = stack1;
        } else if (p == Intake.Pattern.PGP) {
            firstStack = stack1; secondStack = stack3; thirdStack = stack2;
        } else { // PPG or UNKNOWN
            firstStack = stack1; secondStack = stack2; thirdStack = stack3;
        }

        Action routine = drive.actionBuilder(startPose)
                // 1. INITIAL SHOT (Preloads)
                .splineTo(mVec(-17, 0), mAng(180))
                .stopAndAdd(scoreThree)
                
                // 2. CYCLE 1 (First Pixel)
                .setTangent(mAng(45))
                .afterTime(0.5, intakeOn)
                .splineToLinearHeading(new Pose2d(firstStack, mAng(90)), mAng(90))
                .waitSeconds(0.5)
                .afterTime(0, intakeOff)
                .setTangent(mAng(225))
                .splineToLinearHeading(mPose(-17, 12, 180), mAng(180))
                .stopAndAdd(scoreOne)
                
                // 3. CYCLE 2 (Second Pixel)
                .setTangent(mAng(45))
                .afterTime(0.5, intakeOn)
                .splineToLinearHeading(new Pose2d(secondStack, mAng(90)), mAng(90))
                .waitSeconds(0.5)
                .afterTime(0, intakeOff)
                .setTangent(mAng(225))
                .splineToLinearHeading(mPose(-17, 0, 180), mAng(180))
                .stopAndAdd(scoreOne)

                // 4. HIT LEVER & RACK
                .setTangent(mAng(90))
                .splineToLinearHeading(mPose(0, 55, 90), mAng(90))
                .waitSeconds(0.2)
                .setTangent(mAng(180))
                .afterTime(0.1, intakeOn)
                .splineToLinearHeading(mPose(15, 60, 90), mAng(0))
                .waitSeconds(0.8)
                .afterTime(0, intakeOff)
                .setTangent(mAng(180))
                .splineToLinearHeading(mPose(-17, 24, 180), mAng(180))
                .stopAndAdd(scoreThree)
                
                // 5. CYCLE 3 (The Green Ball or Third Pixel)
                .setTangent(mAng(45))
                .afterTime(0.2, intakeOn)
                .splineToLinearHeading(new Pose2d(thirdStack, mAng(90)), mAng(90))
                .waitSeconds(0.5)
                .afterTime(0, intakeOff)
                .setTangent(mAng(225))
                .splineToLinearHeading(mPose(-17, 20, 180), mAng(180))
                .stopAndAdd(scoreOne)
                
                .splineToLinearHeading(mPose(0, 10, 180), mAng(180))
                .build();

        // --- EXECUTION ---
        Actions.runBlocking(new ParallelAction(
                turretTrack,
                flywheelAuto,
                routine
        ));
    }
}

