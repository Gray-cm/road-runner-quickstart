package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class OldRobotIntake {
    private final DcMotorEx spinner;
    private final DcMotorEx boot;
    private final VisionPortal vision;
    private final AprilTagProcessor aprilTag;

    public enum Pattern {
        GPP, // Green 1, Pixel 2, Pixel 3
        PGP, // Pixel 1, Green 2, Pixel 3
        PPG, // Pixel 1, Pixel 2, Green 3
        UNKNOWN
    }

    private Pattern detectedPattern = Pattern.UNKNOWN;

    public OldRobotIntake(HardwareMap hwMap) {
        spinner = hwMap.get(DcMotorEx.class, "spinner");
        boot = hwMap.get(DcMotorEx.class, "boot");
        
        spinner.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        boot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initialize AprilTag processor with default calibration
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        // Initialize VisionPortal - this handles the camera data stream
        vision = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(aprilTag)
                .build();
    }

    public void updatePattern() {
        if (detectedPattern != Pattern.UNKNOWN) return;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                int id = detection.id;
                // ONLY look for pattern tags 1, 2, and 3
                if (id == 21) {
                    detectedPattern = Pattern.GPP;
                    break;
                } else if (id == 22) {
                    detectedPattern = Pattern.PGP;
                    break;
                } else if (id == 23) {
                    detectedPattern = Pattern.PPG;
                    break;
                }
            }
        }
    }

    public void spinning(double power, boolean kick) {
        spinner.setPower(power);
        if (kick) {
            boot.setTargetPosition(boot.getCurrentPosition() + 300);
            boot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            boot.setVelocity(1000);
        }
    }

    public Pattern getDetectedPattern() {
        return detectedPattern;
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTag;
    }

    public void resetPattern() {
        detectedPattern = Pattern.UNKNOWN;
    }

    public void close() {
        if (vision != null) {
            vision.close();
        }
    }
}
