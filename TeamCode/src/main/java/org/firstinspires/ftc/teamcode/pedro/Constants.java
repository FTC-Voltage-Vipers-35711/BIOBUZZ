package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * All Pedro Pathing constants for the robot.
 *
 * HARDWARE (REV Control Hub):
 *   - 4x mecanum drive motors: front_left, back_left, front_right, back_right
 *     (must match the names in the REV configuration exactly)
 *   - goBILDA Pinpoint V2 odometry computer on an I2C port (NOT port 0 - the
 *     Control Hub's built-in IMU lives there). Name it "pinpoint" in the config.
 *   - 2x goBILDA odometry pods plugged INTO the Pinpoint: the forward pod in
 *     the X port, the strafe pod in the Y port.
 *
 * BEFORE RUNNING AN AUTONOMOUS, run the tuners in this order (AutoTune web
 * UI at http://192.168.43.1:10158 while connected to the Robot Controller):
 *   1. Mecanum tuner  - motor directions and drive characterization
 *   2. Pinpoint tuner - pod directions and offsets (fills in localizerConfig)
 *   3. Foresight tuner - path following accuracy (fills in foresightConfig)
 */
public class Constants {

    /** Mecanum drivetrain. Names and directions match the team's REV config. */
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
        c -> {
            c.frontLeftName.set("front_left");
            c.backLeftName.set("back_left");
            c.frontRightName.set("front_right");
            c.backRightName.set("back_right");

            // On this build every direction came out mirrored, so BOTH sides are
            // the opposite of the usual convention (left REVERSE, right FORWARD).
            // This matches Drivetrain.java - keep the two in sync.
            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

            c.manualBrakeMode.set(true);
        }
    );

    /**
     * goBILDA Pinpoint V2 localizer (sensor-fuses the 2 odometry pods with its
     * onboard IMU).
     *
     * The offsets/directions below are PLACEHOLDERS - run the Pinpoint tuner
     * and paste its generated values here. Offsets are in inches, measured from
     * the robot's center of rotation to each pod:
     *   xPodOffset: how far sideways the forward (X) pod is (left = positive)
     *   yPodOffset: how far forward the strafe (Y) pod is (forward = positive)
     */
    public static PinpointConfig localizerConfig = new PinpointConfig(
        c -> {
            c.name.set("pinpoint");
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        }
    );

    /**
     * Foresight - the path-following algorithm. These starting values come from
     * the Pedro docs example robot; run the Foresight tuner and paste the
     * generated values here. The headingFeedback controller is the feedback
     * loop that keeps the robot on the path and driving straight (no curving).
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(
        c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.3);
            Controller secondaryTranslationalForward = Controller.proportional(0.1);
            Controller primaryTranslationalLateral = Controller.proportional(0.3);
            Controller secondaryTranslationalLateral = Controller.proportional(0.1);

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
            c.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
            c.brake.set(Controller.proportionalFeedforward(0.008731598255925491));
            c.headingFeedback.set(Controller.proportional(5.258721785960744));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));
            c.linearBrakeCoefficients.set(Matrix.diag(0.10605894992901523, 0.08719146175596092));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014663966976606565, 0.0013837064502458813));
            c.maxAchievableForwardVelocity.set(72.72923108818539);
            c.maxAchievableStrafeVelocity.set(52.34323936525474);
            c.naturalForwardDeceleration.set(85.01144677379789);
            c.naturalStrafeDeceleration.set(104.49787535782846);
        }
    );

    /** Creates a fully wired Follower (localizer + drivetrain + algorithm). */
    public static Follower create(HardwareMap h) {
        return new Follower(
            new PinpointLocalizer(h, localizerConfig),
            new Mecanum(h, drivetrainConfig),
            new Foresight(foresightConfig)
        );
    }
}
