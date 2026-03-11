package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class Intake {

    private final DcMotorEx spinner;
    private final DcMotor boot;
    private final Servo runner;
    private final Limelight3A limelight;
    private final VoltageSensor voltageSensor;

    private double lastBootPower = 0.0;
    // Base ramp speed.
    private final double baseMaxPowerChange = 0.10;

    private Pattern detectedPattern = Pattern.UNKNOWN;

    public enum Pattern {
        GPP, // Green 1, Pixel 2, Pixel 3
        PGP, // Pixel 1, Green 2, Pixel 3
        PPG, // Pixel 1, Pixel 2, Green 3
        UNKNOWN
    }

    public Intake(HardwareMap hwMap) {
        spinner = hwMap.get(DcMotorEx.class, "spinner");
        boot = hwMap.get(DcMotor.class, "boot");
        limelight = hwMap.get(Limelight3A.class, "limelight");
        runner = hwMap.get(Servo.class, "runner");
        // Initialize voltage sensor
        voltageSensor = hwMap.voltageSensor.iterator().next();
        
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
                if (id == 21) {
                    setPattern(Pattern.GPP);
                    break;
                } else if (id == 22) {
                    setPattern(Pattern.PGP);
                    break;
                } else if (id == 23) {
                    setPattern(Pattern.PPG);
                    break;
                }
            }
        }
    }

    private void setPattern(Pattern newPattern) {
        this.detectedPattern = newPattern;
    }
    public void runit(boolean input){
        if (input) {
            runner.setPosition(0.4);
        }
        else{
            runner.setPosition(0);
        }
    }

    /**
     * Controls the intake spinner and the boot kicker.
     * Uses a voltage-aware slew rate and power limiter for the boot.
     */
    public void runIntakeAndBoot(double intakePower, boolean kick) {
        spinner.setPower(intakePower);
        
        double currentVoltage = getVoltage();
        
        // Voltage Limiter: Scale target power and ramp speed based on battery level.
        // If voltage is low, we decrease max power and ramp even slower to prevent brownouts.
        double voltageScale = Math.max(0.6, Math.min(1.0, currentVoltage / 13.0));
        
        double targetPower = kick ? (-1.0 * voltageScale) : 0.0; // 0.45 being the max allowed power
        
        // Manual slew rate with voltage scaling
        double error = targetPower - lastBootPower;
        double dynamicRamp = baseMaxPowerChange * voltageScale;
        
        if (Math.abs(error) > dynamicRamp) {
            lastBootPower += Math.signum(error) * dynamicRamp;
        } else {
            lastBootPower = targetPower;
        }
        
        boot.setPower(lastBootPower);
    }

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    public double getBootPower() {
        return lastBootPower;
    }

    public Pattern getDetectedPattern() { return detectedPattern; }
    
    public void resetPattern() { detectedPattern = Pattern.UNKNOWN; }
    public void stopp(){
        limelight.close();
    }
}
