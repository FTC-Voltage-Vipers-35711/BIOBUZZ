package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "parralell plate meccanum wheel drive train new build lock in casey", group = "Iterative Opmode")
public class Drivetrain extends OpMode {

    // Stick values smaller than this are ignored so the robot doesn't creep from stick drift
    private static final double STICK_DEADZONE = 0.05;

    // Slow mode floor: left trigger feathers drive speed from full speed (trigger
    // released) down to this value (trigger fully held) for precision driving
    private static final double SLOW_SPEED = 0.35;

    // Heading-hold feedback loop gains (see headingHoldCorrection). The sign of
    // HEADING_KP assumes the current motor directions (left REVERSE / right
    // FORWARD). If you ever flip motor directions again, flip this sign too or
    // the assist will amplify drift instead of fixing it.
    private static final double HEADING_KP = 1.2;              // turn power per radian of error
    private static final double HEADING_KD = 0.06;             // damping per rad/s of rotation
    private static final double HEADING_MAX_CORRECTION = 0.4;  // never fight the driver harder than this

    // Position hold: after the driver leaves both sticks neutral for HOLD_DELAY,
    // the robot locks onto its current field pose and a PD loop drives it back
    // if it gets pushed - same idea as the heading hold, one axis over. The D
    // term comes from the Pinpoint's own velocity estimate, so it's real damping,
    // not a finite difference. Gains are deliberately conservative for a first
    // run; if it overshoots raise POS_KD, if it crawls back raise POS_KP. If it
    // pushes the wrong way when bumped, flip the sign of the strafe line in
    // positionHoldCorrection (drive-motor direction changes shift it).
    private static final double POS_KP = 0.10;                 // power per inch of pose error
    private static final double POS_KD = 0.04;                // power per in/s of drift
    private static final double POS_MAX_CORRECTION = 0.30;    // clamp per axis so a shove can't cause a lunge
    private static final double HOLD_DELAY = 0.25;            // seconds of neutral sticks before locking the pose

    // Slew-rate limiter: motor powers can only change by SLEW_RATE per second,
    // which is what makes starts/stops/hold corrections feel smooth instead of
    // snappy. Applied to the FINAL output, after every other calculation.
    private static final double SLEW_RATE = 6.0;              // power units per second

    // Field-oriented drive: when ON, "up" on the stick always drives up-field
    // no matter which way the robot is rotated. Toggle with the Options button
    // on gamepad1 (the robot rumbles to confirm).
    private static final int TOGGLE_RUMBLE_MS = 150;

    // Declare your 4 drivetrain motors
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;

    // Heading-hold feedback loop (goBILDA Pinpoint V2: 2 odometry pods + IMU).
    // Optional hardware: if the Pinpoint isn't configured yet, the assist
    // quietly disables itself and the teleop drives normally.
    private GoBildaPinpointDriver pinpoint = null;
    private double headingTarget = 0;
    private double lastHeading = 0;
    private final ElapsedTime headingTimer = new ElapsedTime();

    // Field-oriented drive state (see loop()). Always starts in NORMAL
    // robot-oriented mode; press Options to switch to field-oriented.
    private boolean fieldOriented = false;
    private boolean toggleButtonPrev = false;
    private double headingOffset = 0; // heading at Start = "up-field" direction

    // Position-hold state (see loop() and positionHoldCorrection)
    private boolean holdingPose = false;   // true while the PD loop owns the drivetrain
    private double holdX = 0;              // locked field pose, inches
    private double holdY = 0;              // locked field pose, inches
    private final ElapsedTime neutralTimer = new ElapsedTime();

    // Slew-rate limiter state: the last power actually commanded to each motor
    private final double[] lastPower = new double[4]; // FL, BL, FR, BR
    private final ElapsedTime slewTimer = new ElapsedTime();

    @Override
    public void init() {
        // Hardware Mapping - names must perfectly match your REV Control Hub config
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backRight  = hardwareMap.get(DcMotor.class, "back_right");

        // On this build every direction came out mirrored, so BOTH sides are the
        // opposite of the usual convention (left REVERSE, right FORWARD).
        // This inverts forward/back, strafe, AND rotation together.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

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

        // Optional heading sensor for the straight-driving feedback loop
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        } catch (Exception e) {
            pinpoint = null;
        }

        neutralTimer.reset();
        slewTimer.reset();

        telemetry.addData("Status", "Initialized Successfully");
        telemetry.addData("Heading assist", pinpoint != null ? "ON" : "OFF (no pinpoint in config)");
        telemetry.addData("Position hold", pinpoint != null ? "ON (sticks neutral = hold pose)" : "OFF (no pinpoint in config)");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        // Lock the heading-hold target to wherever the robot is actually facing
        // so the assist never yanks the robot toward some old angle.
        if (pinpoint != null) {
            pinpoint.update();
            double heading = pinpoint.getHeading(AngleUnit.RADIANS);
            headingTarget = heading;
            lastHeading = heading;
            // Field frame: "up" on the stick = the way the robot faces right now
            headingOffset = heading;
            headingTimer.reset();
        }

        // Sync the toggle button so a button already held at Start can never
        // silently flip the drive mode - it always starts normal.
        toggleButtonPrev = gamepad1.options;
    }

    @Override
    public void loop() {
        // Field/robot-oriented toggle (Options button on gamepad1)
        boolean toggleButton = gamepad1.options;
        if (toggleButton && !toggleButtonPrev && pinpoint != null) {
            fieldOriented = !fieldOriented;
            gamepad1.rumble(TOGGLE_RUMBLE_MS);
        }
        toggleButtonPrev = toggleButton;

        // Read the Pinpoint once per loop (2 odometry pods + IMU sensor fusion).
        // Heading is CCW-positive radians.
        double heading = 0;
        if (pinpoint != null) {
            pinpoint.update();
            heading = pinpoint.getHeading(AngleUnit.RADIANS);
        }

        // Mecanum math standard: y is forward (negated because push up is negative)
        // x is strafe, rx is rotation
        double y = applyDeadzone(-gamepad1.left_stick_y);
        double x = applyDeadzone(gamepad1.left_stick_x);
        double rx = applyDeadzone(gamepad1.right_stick_x);

        // Field-oriented drive: rotate the stick vector from the field frame
        // into the robot frame so "up" always drives up-field. Rotation stick
        // is left alone - right always turns the robot right in both modes.
        if (fieldOriented) {
            double fieldHeading = heading - headingOffset;
            double cos = Math.cos(fieldHeading);
            double sin = Math.sin(fieldHeading);
            double fieldX = x;
            double fieldY = y;
            x = fieldX * cos + fieldY * sin;
            y = -fieldX * sin + fieldY * cos;
        }

        // Position hold: while nobody is driving, lock onto the pose the robot
        // settled at and PD back to it if it gets pushed. Driving clears the
        // hold; the lock only engages after HOLD_DELAY of neutral sticks so it
        // never fights the driver mid-maneuver or catches settling motion.
        if (pinpoint != null) {
            boolean sticksActive = Math.abs(y) + Math.abs(x) + Math.abs(rx) > 0;
            if (sticksActive) {
                holdingPose = false;
                neutralTimer.reset();
            } else if (!holdingPose && neutralTimer.seconds() > HOLD_DELAY) {
                Pose2D p = pinpoint.getPosition();
                holdX = p.getX(DistanceUnit.INCH);
                holdY = p.getY(DistanceUnit.INCH);
                holdingPose = true;
            }
            if (holdingPose) {
                double[] correction = positionHoldCorrection(heading);
                y += correction[0];  // forward/back
                x += correction[1];  // strafe
            }
        }

        x *= 1.1; // Counteract empirical strafe friction loss (robot's strafe axis)

        // Heading-hold feedback loop: while the driver isn't steering, a PD
        // controller on the Pinpoint heading corrects anything that would make
        // the robot curve off straight (track scrub, uneven motors, bumps).
        rx += headingHoldCorrection(heading, rx);

        // Trigger-tuned slow mode: feather the left trigger to trade speed for
        // precision. Scale is applied after normalization so it only shrinks the
        // output magnitude - steering ratios stay exactly the same.
        double speedScale = 1.0 - (1.0 - SLOW_SPEED) * applyDeadzone(gamepad1.left_trigger);

        // Calculate power for each wheel
        double frontLeftPower  = y + x + rx;
        double backLeftPower   = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower  = y + x - rx;

        // Denominator scaling: ensures powers maintain their proportions
        // if any single calculated power exceeds the motor limit of 1.0
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        // Final output to the hardware, through the slew-rate limiter so every
        // power change (joystick, slow mode, hold corrections) ramps smoothly.
        double[] desired = {
            frontLeftPower  / denominator * speedScale,
            backLeftPower   / denominator * speedScale,
            frontRightPower / denominator * speedScale,
            backRightPower  / denominator * speedScale
        };
        double maxStep = SLEW_RATE * Math.min(slewTimer.seconds(), 0.1); // cap dt so a loop stall can't skip the ramp
        slewTimer.reset();
        for (int i = 0; i < 4; i++) {
            lastPower[i] = Math.max(lastPower[i] - maxStep, Math.min(lastPower[i] + maxStep, desired[i]));
        }
        frontLeft.setPower(lastPower[0]);
        backLeft.setPower(lastPower[1]);
        frontRight.setPower(lastPower[2]);
        backRight.setPower(lastPower[3]);

        // Debugging feedback to Driver Station
        telemetry.addData("Drive mode", driveModeLabel());
        telemetry.addData("Pose hold", holdingPose ? "HOLDING (bump it, it comes back)" : "off (driving)");
        telemetry.addData("Speed scale", "%4.2f", speedScale);
        telemetry.addData("Motors FL | FR", "%4.2f | %4.2f", frontLeft.getPower(), frontRight.getPower());
        telemetry.addData("Motors BL | BR", "%4.2f | %4.2f", backLeft.getPower(), backRight.getPower());
    }

    /**
     * Position-hold PD loop. Returns {forward, strafe} power corrections that
     * drive the robot back to the locked field pose (holdX, holdY). The pose
     * error and the Pinpoint's velocity estimate are both rotated from the
     * field frame into the robot frame so the correction lands on the right
     * wheels regardless of which way the robot is facing.
     */
    private double[] positionHoldCorrection(double heading) {
        Pose2D current = pinpoint.getPosition();
        double errX = holdX - current.getX(DistanceUnit.INCH);
        double errY = holdY - current.getY(DistanceUnit.INCH);

        double velX = pinpoint.getVelX(DistanceUnit.INCH);
        double velY = pinpoint.getVelY(DistanceUnit.INCH);

        // Field frame (x forward, y left at Start) -> robot frame (y forward, x strafe)
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double errForward = errX * cos + errY * sin;
        double errLeft    = -errX * sin + errY * cos;
        double velForward = velX * cos + velY * sin;
        double velLeft    = -velX * sin + velY * cos;

        double forward = POS_KP * errForward - POS_KD * velForward;
        double left    = POS_KP * errLeft    - POS_KD * velLeft;

        forward = Math.max(-POS_MAX_CORRECTION, Math.min(POS_MAX_CORRECTION, forward));
        left    = Math.max(-POS_MAX_CORRECTION, Math.min(POS_MAX_CORRECTION, left));

        telemetry.addData("Pose hold err (in)", "%.2f", Math.hypot(errX, errY));
        return new double[] {forward, -left}; // x stick is right-positive, so negate the left component
    }

    @Override
    public void stop() {
        // Safe-state cleanup when stop is pressed. Null-checked because the SDK can
        // still call stop() if init() failed before the hardware was mapped.
        holdingPose = false;
        for (int i = 0; i < 4; i++) lastPower[i] = 0;
        if (frontLeft != null)  frontLeft.setPower(0);
        if (backLeft != null)   backLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backRight != null)  backRight.setPower(0);
    }

    private static double applyDeadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0 : value;
    }

    /**
     * Heading-hold feedback loop. Returns a turn correction to add to rx so the
     * robot keeps driving straight instead of curving. While the driver is
     * actively steering (rx != 0) the target re-locks to the current heading,
     * so the assist is transparent - it only acts when nobody is steering.
     */
    private double headingHoldCorrection(double heading, double rx) {
        if (pinpoint == null) return 0;

        if (rx != 0) {
            // Driver is steering: follow them and re-lock the target heading
            headingTarget = heading;
            lastHeading = heading;
            headingTimer.reset();
            return 0;
        }

        double dt = headingTimer.seconds();
        headingTimer.reset();

        double error = wrapAngle(headingTarget - heading); // radians, [-pi, pi]
        double velocity = (dt > 0 && dt < 0.5) ? wrapAngle(heading - lastHeading) / dt : 0;
        lastHeading = heading;

        double correction = HEADING_KP * error - HEADING_KD * velocity;
        correction = Math.max(-HEADING_MAX_CORRECTION, Math.min(HEADING_MAX_CORRECTION, correction));

        telemetry.addData("Heading hold err (deg)", "%.1f", Math.toDegrees(error));
        return correction;
    }

    /** Wraps an angle to [-pi, pi] radians so heading errors stay small. */
    private static double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private String driveModeLabel() {
        if (pinpoint == null) return "robot-oriented (no pinpoint in config)";
        return fieldOriented ? "FIELD-oriented (Options toggles)" : "robot-oriented (Options toggles)";
    }
}
