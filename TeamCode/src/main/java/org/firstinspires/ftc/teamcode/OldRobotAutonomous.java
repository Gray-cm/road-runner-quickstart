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
 * Abstract base class for high-speed autonomous on the older robot.
 * Movement is calculated using Wheel Encoders (OldRobotMecanumDrive).
 */
public abstract class OldRobotAutonomous extends LinearOpMode {
    public enum Alliance { RED, BLUE }
    protected abstract Alliance getAlliance();

    private OldRobotIntake intake;
    private OldRobotFlywheel flywheel;
    private OldRobotMecanumDrive drive;

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
        intake = new OldRobotIntake(hardwareMap);
        flywheel = new OldRobotFlywheel(hardwareMap);
        
        Pose2d startPose = mPose(60, 0, 180);
        drive = new OldRobotMecanumDrive(hardwareMap, startPose);

        // Set correct target ID based on alliance
        flywheel.setTargetTagId(getAlliance() == Alliance.BLUE ? 20 : 24);

        // --- BACKGROUND ACTIONS ---
        Action autoShoot = packet -> {
            flywheel.update(flywheel.getAutoVelocity());
            return true; 
        };

        // --- SCORING ACTION ---
        Action scoreThree = new SequentialAction(
                packet -> { intake.spinning(0, true); return false; },
                new SleepAction(0.5),
                packet -> { intake.spinning(0, false); return false; },
                new SleepAction(0.2),
                packet -> { intake.spinning(0, true); return false; },
                new SleepAction(0.5),
                packet -> { intake.spinning(0, false); return false; }
        );

        Action intakeOn = packet -> { intake.spinning(1.0, false); return false; };
        Action intakeOff = packet -> { intake.spinning(0.0, false); return false; };

        // --- INIT ---
        while (!isStarted() && !isStopRequested()) {
            intake.updatePattern();
            telemetry.addData("Alliance", getAlliance());
            telemetry.addData("Pattern", intake.getDetectedPattern());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // --- DYNAMIC INTAKE ORDER ---
        Vector2d stack1 = mVec(12, 48);
        Vector2d stack2 = mVec(36, 48);
        Vector2d stack3 = mVec(-11.3, 48);
        
        Vector2d firstStack, secondStack, thirdStack;
        OldRobotIntake.Pattern p = intake.getDetectedPattern();
        
        if (p == OldRobotIntake.Pattern.GPP) {
            firstStack = stack2; secondStack = stack3; thirdStack = stack1;
        } else if (p == OldRobotIntake.Pattern.PGP) {
            firstStack = stack1; secondStack = stack3; thirdStack = stack2;
        } else { 
            firstStack = stack1; secondStack = stack2; thirdStack = stack3;
        }

        Action routine = drive.actionBuilder(startPose)
                .splineTo(mVec(-17, 0), mAng(180))
                .stopAndAdd(scoreThree)
                .setTangent(mAng(45))
                .afterTime(0.5, intakeOn)
                .splineToLinearHeading(new Pose2d(firstStack, mAng(90)), mAng(90))
                .waitSeconds(0.5)
                .afterTime(0, intakeOff)
                .setTangent(mAng(225))
                .splineToLinearHeading(mPose(-17, 12, 180), mAng(180))
                .stopAndAdd(scoreThree)
                .build();

        Actions.runBlocking(new ParallelAction(
                autoShoot,
                routine
        ));
        
        intake.close();
        flywheel.close();
    }
}
