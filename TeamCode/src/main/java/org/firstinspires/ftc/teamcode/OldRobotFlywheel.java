package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class OldRobotFlywheel {
    private final DcMotorEx shoot1;
    private final DcMotorEx shoot2;
    private final PIDFController flywheelPID;
    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTag;
    private final InterpLUT velocityLUT;
    
    private int targetTagId = -1;

    private static final double Kp = 0.005;
    private static final double Ki = 0.0;
    private static final double Kd = 0.0;
    private static final double Kf = 0.00015;

    public OldRobotFlywheel(HardwareMap hardwareMap) {
        shoot1 = hardwareMap.get(DcMotorEx.class, "Shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "Shoot2");
        
        shoot1.setDirection(DcMotorEx.Direction.REVERSE);
        shoot2.setDirection(DcMotorEx.Direction.REVERSE);

        shoot1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shoot2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        
        flywheelPID = new PIDFController(Kp, Ki, Kd, Kf);

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(aprilTag)
                .build();

        velocityLUT = new InterpLUT();
        velocityLUT.add(20.0, 1750.0);
        velocityLUT.add(30.0, 1900.0);
        velocityLUT.add(40.0, 2050.0);
        velocityLUT.add(50.0, 2200.0);
        velocityLUT.add(60.0, 2350.0);
        velocityLUT.add(70.0, 2500.0);
        velocityLUT.createLUT();
    }

    public void setTargetTagId(int id) {
        this.targetTagId = id;
    }

    public int getTargetTagId() {
        return targetTagId;
    }

    public AprilTagDetection getTargetDetection() {
        if (targetTagId == -1) return null;
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.id == targetTagId) {
                return detection;
            }
        }
        return null;
    }

    public void update(double targetVelocity) {
        if (targetVelocity <= 0.05) {
            shoot1.setPower(0);
            shoot2.setPower(0);
            return;
        }

        if (targetVelocity <= 1.0) {
            shoot1.setPower(targetVelocity);
            shoot2.setPower(targetVelocity);
        } else {
            double currentVelocity = shoot1.getVelocity();
            double power = flywheelPID.calculate(currentVelocity, targetVelocity);
            power = Math.max(-1.0, Math.min(1.0, power));
            shoot1.setPower(power);
            shoot2.setPower(power);
        }
    }

    public double getAutoVelocity() {
        AprilTagDetection detection = getTargetDetection();
        if (detection != null && detection.ftcPose != null) {
            return velocityLUT.get(detection.ftcPose.range);
        }
        return 2000; 
    }

    public double getVelocity() {
        return shoot1.getVelocity();
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
