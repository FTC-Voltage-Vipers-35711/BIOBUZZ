package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "parralell plate meccanum wheel drive train new build lock in casey", group = "Iterative Opmode")
public class Drivetrain extends OpMode {

    // Stick values smaller than this are ignored so the robot doesn't creep from stick drift
    private static final double STICK_DEADZONE = 0.05;

    // Declare your 4 drivetrain motors
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;

    @Override
    public void init() {
        // Hardware Mapping - names must perfectly match your REV Control Hub config
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backRight  = hardwareMap.get(DcMotor.class, "back_right");

        // Reverse the right side motors so positive power moves the robot forward
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Open-loop power control - don't let a leftover encoder mode from another
        // OpMode change how setPower behaves
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Optional: Brake mode stops the robot immediately when joysticks are released
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized Successfully");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        // Mecanum math standard: y is forward (negated because push up is negative)
        // x is strafe, rx is rotation
        double y = applyDeadzone(-gamepad1.left_stick_y);
        double x = applyDeadzone(gamepad1.left_stick_x) * 1.1; // Counteract empirical strafe friction loss
        double rx = applyDeadzone(gamepad1.right_stick_x);

        // Calculate power for each wheel
        double frontLeftPower  = y + x + rx;
        double backLeftPower   = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower  = y + x - rx;

        // Denominator scaling: ensures powers maintain their proportions
        // if any single calculated power exceeds the motor limit of 1.0
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        // Final output to the hardware
        frontLeft.setPower(frontLeftPower / denominator);
        backLeft.setPower(backLeftPower / denominator);
        frontRight.setPower(frontRightPower / denominator);
        backRight.setPower(backRightPower / denominator);

        // Debugging feedback to Driver Station
        telemetry.addData("Motors FL | FR", "%4.2f | %4.2f", frontLeft.getPower(), frontRight.getPower());
        telemetry.addData("Motors BL | BR", "%4.2f | %4.2f", backLeft.getPower(), backRight.getPower());
    }

    @Override
    public void stop() {
        // Safe-state cleanup when stop is pressed. Null-checked because the SDK can
        // still call stop() if init() failed before the hardware was mapped.
        if (frontLeft != null)  frontLeft.setPower(0);
        if (backLeft != null)   backLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backRight != null)  backRight.setPower(0);
    }

    private static double applyDeadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0 : value;
    }
}
