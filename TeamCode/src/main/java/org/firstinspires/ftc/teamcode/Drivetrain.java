package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private final PIDFController flPIDF;
    private final PIDFController frPIDF;
    private final PIDFController blPIDF;
    private final PIDFController brPIDF;
    
    private static final double TICKS_PER_METER = 1500; //will need tuning
    private final DcMotorEx fl;
    private final DcMotorEx fr;
    private final DcMotorEx br;
    private final DcMotorEx bl;
    
    private double Kp = 0.05;
    private double Ki = 0.025;
    private double Kd = 0.04;
    private double Kf = 0;

    public Drivetrain(HardwareMap hwMap){
        fl = hwMap.get(DcMotorEx.class, "forwardleft_motor");
        fr = hwMap.get(DcMotorEx.class, "forwardright_motor");
        br = hwMap.get(DcMotorEx.class, "backright_motor");
        bl = hwMap.get(DcMotorEx.class, "backleft_motor");
        
        fl.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);
        
        fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flPIDF = new PIDFController(Kp, Ki, Kd, Kf);
        frPIDF = new PIDFController(Kp, Ki, Kd, Kf);
        blPIDF = new PIDFController(Kp, Ki, Kd, Kf);
        brPIDF = new PIDFController(Kp, Ki, Kd, Kf);
    }
    
    public void drive(double power, double strafe, double turn){
        double flTarget = power + strafe + turn;
        double frTarget = power - strafe - turn;
        double blTarget = power - strafe + turn;
        double brTarget = power + strafe - turn;
        
        fl.setPower(flPIDF.calculate(getFLVelocity(), flTarget));
        fr.setPower(frPIDF.calculate(getFRVelocity(), frTarget));
        bl.setPower(blPIDF.calculate(getBLVelocity(), blTarget));
        br.setPower(brPIDF.calculate(getBRVelocity(), brTarget));
    }
    
    public void AutonomousTrap(double maxV, double maxA, double goalPos, double goalFv, double initPos, double initFv){
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxV,maxA);
        TrapezoidProfile.State goal = new TrapezoidProfile.State(goalPos,goalFv);
        TrapezoidProfile.State init = new TrapezoidProfile.State(initPos,initFv);
        TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, init);
        
        double kP = 0.05;
        double kI = 0.001;
        double kD = 0.025;
        ProfiledPIDController controller = new ProfiledPIDController(kP,kI,kD,constraints);
        
        fl.setVelocity(controller.calculate(fl.getCurrentPosition(), goal));
        bl.setVelocity(controller.calculate(bl.getCurrentPosition(), goal));
        fr.setVelocity(controller.calculate(fr.getCurrentPosition(), goal));
        br.setVelocity(controller.calculate(br.getCurrentPosition(), goal));
    }
    
    public void stopMotors(){
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    
    public double getFLVelocity(){
        return fl.getVelocity() / TICKS_PER_METER; //mps
    }
    
    public double getFRVelocity(){
        return fr.getVelocity() / TICKS_PER_METER; //mps
    }
    
    public double getBLVelocity(){
        return bl.getVelocity() / TICKS_PER_METER; //mps
    }
    
    public double getBRVelocity(){
        return br.getVelocity() / TICKS_PER_METER; //mps
    }

    public double getKp() { return Kp; }
    public void setKp(double kp) { this.Kp = kp; updatePIDF(); }

    public double getKi() { return Ki; }
    public void setKi(double ki) { this.Ki = ki; updatePIDF(); }

    public double getKd() { return Kd; }
    public void setKd(double kd) { this.Kd = kd; updatePIDF(); }

    public double getKf() { return Kf; }
    public void setKf(double kf) { this.Kf = kf; updatePIDF(); }

    private void updatePIDF() {
        flPIDF.setPIDF(Kp, Ki, Kd, Kf);
        frPIDF.setPIDF(Kp, Ki, Kd, Kf);
        blPIDF.setPIDF(Kp, Ki, Kd, Kf);
        brPIDF.setPIDF(Kp, Ki, Kd, Kf);
    }
}
