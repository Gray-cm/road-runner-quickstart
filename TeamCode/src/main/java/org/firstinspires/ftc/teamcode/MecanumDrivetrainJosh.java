/*package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class MecanumDrivetrainJosh {
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    // 1. Hardware Mapping
    // Ensure these names match your configuration on the Driver Station!
    public MecanumDrivetrainJosh(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.dcMotor.get("forwardright_motor");
        backLeft = hardwareMap.dcMotor.get("backleft_motor");
        frontRight = hardwareMap.dcMotor.get("forwardRight_motor");
        backRight = hardwareMap.dcMotor.get("backright_motor");

        // 2. Reverse left side motors
        // Most FTC drivetrains need one side reversed to drive forward together
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // 3. Get Joystick Inputs
    double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed by default
    double x = gamepad1.left_stick_x * 1.1; // Multiplier to counteract friction in strafing
    double rx = gamepad1.right_stick_x;

    // 4. Mecanum Math
    // Denominator is the largest motor power (absolute value) or 1
    // This maintains the ratio of power between motors
    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
    double frontLeftPower = (y + x + rx) / denominator;
    double backLeftPower = (y - x + rx) / denominator;
    double frontRightPower = (y - x - rx) / denominator;
    double backRightPower = (y + x - rx) / denominator;

    public void driving(){
    // 5. Set Motor Powers
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
}
        }
*/
