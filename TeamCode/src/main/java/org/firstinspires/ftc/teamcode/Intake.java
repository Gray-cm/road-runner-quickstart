package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private final DcMotorEx spinner;
    private final Servo boot;
    private final Limelight3A limelight;

    private Pattern detectedPattern = Pattern.UNKNOWN;

    public enum Pattern {
        GPP, // Green 1, Pixel 2, Pixel 3
        PGP, // Pixel 1, Green 2, Pixel 3
        PPG, // Pixel 1, Pixel 2, Green 3
        UNKNOWN
    }

    public Intake(HardwareMap hwMap) {
        spinner = hwMap.get(DcMotorEx.class, "spinner");
        boot = hwMap.get(Servo.class, "boot");
        limelight = hwMap.get(Limelight3A.class, "limelight");
        
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);
        spinner.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Ensure Limelight is started
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void updatePattern() {
        if (detectedPattern != Pattern.UNKNOWN) return;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                int id = fr.getFiducialId();
                // ONLY look for pattern tags 1, 2, and 3 to avoid goal conflicts
                if (id == 1) {
                    setPattern(Pattern.GPP);
                    break;
                } else if (id == 2) {
                    setPattern(Pattern.PGP);
                    break;
                } else if (id == 3) {
                    setPattern(Pattern.PPG);
                    break;
                }
            }
        }
    }

    private void setPattern(Pattern newPattern) {
        this.detectedPattern = newPattern;
    }

    public void runIntakeAndBoot(double intakePower, boolean kick) {
        spinner.setPower(intakePower);
        // Kick logic
        boot.setPosition(kick ? 1.0 : 0.5);
    }

    public Pattern getDetectedPattern() { return detectedPattern; }
    
    public void resetPattern() { detectedPattern = Pattern.UNKNOWN; }
}
