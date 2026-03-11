package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class Flywheel {
    // Flywheel PIDF Constants
    private final double kp = 0.655;
    private final double ki = 0.35;
    private final double kd = 0.05;
    private final double kf = 0.20;

    // Turret PID Constants (for alignment using tx)
    private final double kpi = 0.065; // Increased gain for better response //Originally 0.045
    private final double kii = 0.001;
    private final double kdi = 0.005; // originally 0.002
    private final double kStaticTurret = 0.08;
    private final DcMotorEx shoot1;
    private final DcMotorEx shoot2;
    private final CRServo spin;
    private final Limelight3A limelight;
    private final PIDFController flywheelPID;
    private final PIDFController turretPID;

    private int targetTagId = -1;
    public Flywheel(HardwareMap hardwareMap) {
        shoot1 = hardwareMap.get(DcMotorEx.class, "Shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "Shoot2");
        spin = hardwareMap.get(CRServo.class, "spin");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        shoot1.setDirection(DcMotorEx.Direction.FORWARD);
        shoot2.setDirection(DcMotorEx.Direction.FORWARD);

        shoot1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shoot2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        
        flywheelPID = new PIDFController(kp, ki, kd, kf);
        turretPID = new PIDFController(kpi, kii, kdi, 0);
        
        limelight.pipelineSwitch(0); 
        limelight.start();
    }

    public void setTargetTagId(int id) {
        this.targetTagId = id;
    }

    /**
     * update() solely controls the motor power to reach a target velocity.
     */
    public void update(double targetVelocity) {
        if (targetVelocity <= 0.05) { // Small deadzone for trigger
            shoot1.setPower(0);
            shoot2.setPower(0);
            return;
        }

        // In TeleOp, targetVelocity is the gamepad trigger (0 to 1).
        // In Auto, targetVelocity is the RPM/Ticks (e.g., 2000).

        if (targetVelocity <= 1.0) {
            // TELEOP MODE: Power-based control
            shoot1.setPower(targetVelocity);
            shoot2.setPower(targetVelocity);
        } else {
            // AUTO MODE: PID Velocity-based control
            double power = flywheelPID.calculate(shoot1.getVelocity(), targetVelocity);
            shoot1.setPower(power);
            shoot2.setPower(power);
        }
    }

    /**
     * spinning() solely controls the TURRET direction using tx.
     */
    private LLResult lastResult; // Add this as a global variable in the class

    public void updateLimelightData() {
        lastResult = limelight.getLatestResult();
    }
    public void spinning() {
        updateLimelightData();
        if (lastResult != null && lastResult.isValid()) {
            double tx = 0;
            boolean found = false;

            // Search for specific target ID if set
            if (targetTagId != -1) {
                for (LLResultTypes.FiducialResult fiducial : lastResult.getFiducialResults()) {
                    if (fiducial.getFiducialId() == targetTagId) {
                        tx = fiducial.getTargetXDegrees();
                        found = true;
                        break;
                    }
                }
            } else {
                // Fallback to primary result if no specific ID target
                tx = lastResult.getTx();
                found = true;
            }

            if (found) {
                double setpoint = 5;
                // Aim to center the turret (tx = 0)
                double spinPower = turretPID.calculate(tx, setpoint); // the second argument is for offset, 0 for perfect alignment of the limelight and code.negative for right offset and positive for left offset
                if (Math.abs(setpoint - tx) > 0.5) {
                    spinPower += Math.signum(spinPower) * kStaticTurret;
                }
                spin.setPower(spinPower);
            }
        } else {
            spin.setPower(0);
        }
    }

    /**
     * Calculates the required flywheel velocity based on distance (ty).
     */
    public double getAutoVelocity() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = 0;
            boolean found = false;

            if (targetTagId != -1) {
                for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                    if (fiducial.getFiducialId() == targetTagId) {
                        ty = fiducial.getTargetYDegrees();
                        found = true;
                        break;
                    }
                }
            } else {
                ty = result.getTy();
                found = true;
            }

            if (found) {
                // Tuning required: Map vertical offset (distance) to velocity
                return 3500 + (ty * -100);
            }
        }
        return 2000; // Default safety velocity
    }

    public void autoShoot() {
        update(getAutoVelocity());
    }

    public double getVelocity() {
        return shoot1.getVelocity();
    }
    public void stopp(){
        limelight.close();
    }
}
